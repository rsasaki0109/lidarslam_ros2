#!/usr/bin/env python3
"""Find public MID-360 datasets that can exercise loop-alignment checks."""

from __future__ import annotations

import hashlib
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_loop_alignment_analyzer import (
    LoopAlignmentThresholds,
    find_loop_candidates,
    find_nearest_revisit,
    parse_tum_trajectory_lines,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_LOOP_CANDIDATES_JSON = 'mid360_robot_public_loop_candidates.json'
PUBLIC_LOOP_CANDIDATES_MARKDOWN = 'mid360_robot_public_loop_candidates.md'

HARD_POINTCLOUD_RECORD_URL = 'https://zenodo.org/records/10122133'
HARD_POINTCLOUD_GT_URL = (
    'https://zenodo.org/api/records/10122133/files/gt.zip/content'
)
HARD_POINTCLOUD_GT_MD5 = 'ab2a80dc1d06767b7c3ce1893f3020b0'

HARD_POINTCLOUD_SEQUENCE_BAGS: dict[str, dict[str, Any]] = {
    'outdoor_kidnap': {
        'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
        'sensor': 'Livox MID360',
        'pointcloud_topic': '/livox/points',
        'imu_topic': '/livox/imu',
        'bag_file_ids': ['outdoor_kidnap_a', 'outdoor_kidnap_b'],
        'bag_filenames': ['outdoor_kidnap_a.zip', 'outdoor_kidnap_b.zip'],
        'notes': 'Split outdoor MID360 sequence; replay or merge a then b for a full loop.',
    },
    'outdoor_hard_01': {
        'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
        'sensor': 'Livox MID360',
        'pointcloud_topic': '/livox/points',
        'imu_topic': '/livox/imu',
        'bag_file_ids': ['outdoor_hard_01a', 'outdoor_hard_01b'],
        'bag_filenames': ['outdoor_hard_01a.zip', 'outdoor_hard_01b.zip'],
        'notes': 'Split outdoor MID360 hard-motion sequence.',
    },
    'outdoor_hard_02': {
        'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
        'sensor': 'Livox MID360',
        'pointcloud_topic': '/livox/points',
        'imu_topic': '/livox/imu',
        'bag_file_ids': ['outdoor_hard_02a', 'outdoor_hard_02b'],
        'bag_filenames': ['outdoor_hard_02a.zip', 'outdoor_hard_02b.zip'],
        'notes': 'Split outdoor MID360 hard-motion sequence.',
    },
}


@dataclass(frozen=True)
class PublicLoopCandidateOptions:
    """Options for public loop-candidate analysis."""

    dataset_root: Path
    output_dir: Path
    gt_zip: Path | None = None
    download_gt: bool = False
    verify_md5: bool = True
    include_indoor: bool = False
    thresholds: LoopAlignmentThresholds = LoopAlignmentThresholds()
    max_loop_candidates: int = 20


class PublicLoopCandidateAnalyzer:
    """Analyze lightweight public GT trajectories before downloading large bags."""

    def analyze(self, options: PublicLoopCandidateOptions) -> dict[str, Any]:
        """Build a public loop-candidate report."""
        gt_zip = _resolve_gt_zip(options)
        messages: list[str] = []
        if options.download_gt:
            _download_gt_zip(gt_zip, messages)
        if options.verify_md5 and gt_zip.is_file():
            _verify_md5(gt_zip, HARD_POINTCLOUD_GT_MD5)
            messages.append(f'MD5 verified for {gt_zip.name}.')

        sequences = _analyze_gt_zip(
            gt_zip=gt_zip,
            thresholds=options.thresholds,
            include_indoor=options.include_indoor,
            max_loop_candidates=max(1, options.max_loop_candidates),
        )
        recommended = [
            sequence for sequence in sequences
            if sequence['status'] == 'LOOP_CANDIDATE' and sequence['target_mid360']
        ]
        status = 'PASS' if recommended else 'WARN'
        if not gt_zip.is_file():
            status = 'FAIL'
            messages.append('GT zip is missing; pass --download-gt or --gt-zip.')

        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'source': {
                'title': 'Hard Point Cloud Localization Dataset',
                'record_url': HARD_POINTCLOUD_RECORD_URL,
                'gt_url': HARD_POINTCLOUD_GT_URL,
                'gt_md5': HARD_POINTCLOUD_GT_MD5,
            },
            'gt_zip': str(gt_zip),
            'output_dir': str(options.output_dir.expanduser().resolve()),
            'thresholds': asdict(options.thresholds),
            'counts': _counts(sequences),
            'recommended_sequences': recommended,
            'sequences': sequences,
            'messages': messages,
            'next_actions': _next_actions(recommended),
        }
        return report


def write_public_loop_candidate_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write public loop-candidate JSON and Markdown artifacts."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PUBLIC_LOOP_CANDIDATES_JSON
    markdown_path = output_dir / PUBLIC_LOOP_CANDIDATES_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_public_loop_candidate_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_public_loop_candidate_markdown(report: dict[str, Any]) -> str:
    """Render public loop-candidate analysis as Markdown."""
    lines = [
        '# MID-360 Public Loop Candidates',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- gt_zip: `{report.get('gt_zip', '')}`",
        f"- loop_search_radius_m: `{report.get('thresholds', {}).get('loop_search_radius_m')}`",
        f"- min_time_separation_sec: "
        f"`{report.get('thresholds', {}).get('min_time_separation_sec')}`",
        '',
        '## Recommended Sequences',
        '',
    ]
    recommended = report.get('recommended_sequences') or []
    if recommended:
        for item in recommended:
            lines.append(
                f"- `{item['sequence_id']}`: nearest `{_fmt(item.get('nearest_revisit_distance_m'))}` m, "
                f"loop candidates `{item.get('loop_candidate_count')}`, "
                f"bags `{', '.join(item.get('bag_filenames') or [])}`"
            )
    else:
        lines.append('- none')

    lines.extend(['', '## Sequences', ''])
    for item in report.get('sequences') or []:
        lines.append(
            f"- `{item['status']}` `{item['sequence_id']}`: "
            f"poses `{item.get('poses')}`, duration `{_fmt(item.get('duration_sec'))}` s, "
            f"nearest `{_fmt(item.get('nearest_revisit_distance_m'))}` m, "
            f"candidates `{item.get('loop_candidate_count')}`"
        )

    lines.extend(['', '## Next Actions', ''])
    actions = report.get('next_actions') or []
    if actions:
        lines.extend(f'- {action}' for action in actions)
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _resolve_gt_zip(options: PublicLoopCandidateOptions) -> Path:
    if options.gt_zip:
        return options.gt_zip.expanduser().resolve()
    return (
        options.dataset_root.expanduser().resolve()
        / 'hard_pointcloud_mid360_outdoor_kidnap_a'
        / 'archives'
        / 'gt.zip'
    )


def _download_gt_zip(path: Path, messages: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file():
        messages.append(f'GT zip already exists: {path}')
        return
    part_path = path.with_suffix(path.suffix + '.part')
    if part_path.exists():
        part_path.unlink()
    with urllib.request.urlopen(HARD_POINTCLOUD_GT_URL, timeout=60) as response:
        part_path.write_bytes(response.read())
    part_path.replace(path)
    messages.append(f'Downloaded GT zip: {path}')


def _verify_md5(path: Path, expected_md5: str) -> None:
    md5 = hashlib.md5()
    with path.open('rb') as input_file:
        while True:
            chunk = input_file.read(1024 * 1024)
            if not chunk:
                break
            md5.update(chunk)
    actual = md5.hexdigest()
    if actual.lower() != expected_md5.lower():
        raise ValueError(f'MD5 mismatch for {path}: expected {expected_md5}, got {actual}')


def _analyze_gt_zip(
    *,
    gt_zip: Path,
    thresholds: LoopAlignmentThresholds,
    include_indoor: bool,
    max_loop_candidates: int,
) -> list[dict[str, Any]]:
    if not gt_zip.is_file():
        return []
    sequences = []
    with zipfile.ZipFile(gt_zip) as archive:
        names = sorted(
            name for name in archive.namelist()
            if name.startswith('gt/traj_lidar_') and name.endswith('.txt')
        )
        for name in names:
            sequence_id = Path(name).stem.removeprefix('traj_lidar_')
            target_mid360 = sequence_id.startswith('outdoor_')
            if not target_mid360 and not include_indoor:
                continue
            text = archive.read(name).decode('utf-8', errors='replace')
            poses = parse_tum_trajectory_lines(text.splitlines())
            nearest = find_nearest_revisit(poses, thresholds)
            candidates = find_loop_candidates(
                poses,
                thresholds,
                max_candidates=max_loop_candidates,
            )
            metadata = HARD_POINTCLOUD_SEQUENCE_BAGS.get(sequence_id, {})
            status = 'LOOP_CANDIDATE' if target_mid360 and candidates else 'NO_LOOP_WITHIN_RADIUS'
            if not target_mid360:
                status = 'REFERENCE_ONLY'
            sequences.append({
                'sequence_id': sequence_id,
                'status': status,
                'target_mid360': target_mid360,
                'trajectory_file': name,
                'poses': len(poses),
                'duration_sec': _duration(poses),
                'path_length_m': _path_length(poses),
                'nearest_revisit_distance_m': (
                    nearest.get('distance_m') if nearest else None
                ),
                'loop_candidate_count': len(candidates),
                'loop_candidates': candidates,
                'dataset_id': metadata.get('dataset_id', ''),
                'sensor': metadata.get('sensor', 'non-MID360 reference'),
                'pointcloud_topic': metadata.get('pointcloud_topic', ''),
                'imu_topic': metadata.get('imu_topic', ''),
                'bag_file_ids': metadata.get('bag_file_ids', []),
                'bag_filenames': metadata.get('bag_filenames', []),
                'notes': metadata.get('notes', ''),
            })
    sequences.sort(
        key=lambda item: (
            0 if item['status'] == 'LOOP_CANDIDATE' else 1,
            0 if item['target_mid360'] else 1,
            float(item.get('nearest_revisit_distance_m') or 1e9),
            item['sequence_id'],
        )
    )
    return sequences


def _counts(sequences: list[dict[str, Any]]) -> dict[str, int]:
    return {
        'total': len(sequences),
        'target_mid360': sum(1 for item in sequences if item['target_mid360']),
        'loop_candidate': sum(1 for item in sequences if item['status'] == 'LOOP_CANDIDATE'),
        'no_loop_within_radius': sum(
            1 for item in sequences if item['status'] == 'NO_LOOP_WITHIN_RADIUS'
        ),
    }


def _next_actions(recommended: list[dict[str, Any]]) -> list[str]:
    if not recommended:
        return [
            'Use a larger loop radius only for diagnosis, or find another public MID-360 bag with a real revisit.',
        ]
    actions = []
    for item in recommended[:3]:
        file_ids = ', '.join(item.get('bag_file_ids') or [])
        actions.append(
            f'Download `{item["sequence_id"]}` split bags in order ({file_ids}) before running SLAM.'
        )
    actions.append(
        'Merge or replay the split rosbags in sequence, then run RKO-LIO and the loop-alignment cloud analyzer.'
    )
    return actions


def _duration(poses: list[Any]) -> float:
    if len(poses) < 2:
        return 0.0
    return float(poses[-1].stamp - poses[0].stamp)


def _path_length(poses: list[Any]) -> float:
    total = 0.0
    for prev, cur in zip(poses, poses[1:]):
        total += (
            (prev.x - cur.x) ** 2
            + (prev.y - cur.y) ** 2
            + (prev.z - cur.z) ** 2
        ) ** 0.5
    return total


def _fmt(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.3f}'
    except Exception:
        return ''
