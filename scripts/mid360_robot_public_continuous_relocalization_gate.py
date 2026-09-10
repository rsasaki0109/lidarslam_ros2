#!/usr/bin/env python3
"""Completion gate for public MID-360 continuous RKO-LIO kidnap relocalization."""

from __future__ import annotations

import contextlib
import io
import json
import re
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json
from lidarslam_benchmark_tools.verify_autoware_map import MapVerifier


CONTINUOUS_RELOCALIZATION_GATE_JSON = (
    'mid360_robot_public_continuous_relocalization_gate.json'
)
CONTINUOUS_RELOCALIZATION_GATE_MARKDOWN = (
    'mid360_robot_public_continuous_relocalization_gate.md'
)
DEFAULT_PUBLIC_LOOP_START_STAMP_SEC = 1693922461.499998
DEFAULT_PUBLIC_LOOP_END_STAMP_SEC = 1693922994.700686


@dataclass(frozen=True)
class ContinuousRelocalizationGateOptions:
    """Inputs for continuous public MID-360 relocalization evidence."""

    run_dir: Path
    output_dir: Path
    loop_alignment_json: Path
    tracked_rko_config: Path
    min_rko_poses: int = 1000
    min_trajectory_duration_sec: float = 500.0
    min_relocalization_events: int = 1
    max_loop_distance_m: float = 1.0
    gt_loop_start_stamp_sec: float = DEFAULT_PUBLIC_LOOP_START_STAMP_SEC
    gt_loop_end_stamp_sec: float = DEFAULT_PUBLIC_LOOP_END_STAMP_SEC
    max_public_endpoint_distance_m: float = 5.0
    max_public_endpoint_stamp_error_sec: float = 1.0
    scope: str = 'public_mid360_continuous_rko_lio_kidnap_relocalization'


class ContinuousRelocalizationGate:
    """Build a PASS/FAIL report for continuous kidnap relocalization."""

    def build_report(self, options: ContinuousRelocalizationGateOptions) -> dict[str, Any]:
        """Build and write a continuous relocalization gate report."""
        run_dir = options.run_dir.expanduser().resolve()
        output_dir = options.output_dir.expanduser().resolve()
        loop_alignment = _load_json(options.loop_alignment_json)
        trajectory = _trajectory_summary(run_dir)
        public_endpoint = _public_loop_endpoint_summary(
            run_dir,
            start_stamp_sec=float(options.gt_loop_start_stamp_sec),
            end_stamp_sec=float(options.gt_loop_end_stamp_sec),
        )
        autoware_verify = _verify_autoware_map(run_dir)
        recovery = _recovery_summary(run_dir / 'slam.launch.log')
        config = _config_summary(run_dir, options.tracked_rko_config)
        checks = _checks(
            trajectory=trajectory,
            autoware_verify=autoware_verify,
            loop_alignment=loop_alignment,
            public_endpoint=public_endpoint,
            recovery=recovery,
            config=config,
            min_rko_poses=max(1, int(options.min_rko_poses)),
            min_trajectory_duration_sec=max(0.0, float(options.min_trajectory_duration_sec)),
            min_relocalization_events=max(0, int(options.min_relocalization_events)),
            max_loop_distance_m=max(0.0, float(options.max_loop_distance_m)),
            max_public_endpoint_distance_m=max(
                0.0, float(options.max_public_endpoint_distance_m)
            ),
            max_public_endpoint_stamp_error_sec=max(
                0.0, float(options.max_public_endpoint_stamp_error_sec)
            ),
        )
        status = _overall_status(checks)
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'scope': options.scope,
            'completion_ready': status == 'PASS',
            'output_dir': str(output_dir),
            'options': _options_payload(options),
            'artifacts': {
                'run_dir': str(run_dir),
                'loop_alignment_json': str(options.loop_alignment_json.expanduser().resolve()),
                'tracked_rko_config': str(options.tracked_rko_config.expanduser().resolve()),
                'slam_launch_log': str(run_dir / 'slam.launch.log'),
            },
            'evidence': {
                'trajectory': trajectory,
                'autoware_map_verify': autoware_verify,
                'loop_alignment': _loop_alignment_evidence(loop_alignment),
                'public_loop_endpoint': public_endpoint,
                'recovery': recovery,
                'config': config,
            },
            'checks': checks,
            'counts': _count_checks(checks),
            'next_actions': _next_actions(checks),
        }
        write_continuous_relocalization_gate_report(report, output_dir)
        return report


def write_continuous_relocalization_gate_report(
    report: dict[str, Any],
    output_dir: Path,
) -> dict[str, Path]:
    """Write JSON and Markdown reports."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / CONTINUOUS_RELOCALIZATION_GATE_JSON
    markdown_path = output_dir / CONTINUOUS_RELOCALIZATION_GATE_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(
        render_continuous_relocalization_gate_markdown(report) + '\n',
        encoding='utf-8',
    )
    return {'json': json_path, 'markdown': markdown_path}


def render_continuous_relocalization_gate_markdown(report: dict[str, Any]) -> str:
    """Render a concise Markdown report."""
    evidence = report.get('evidence') or {}
    trajectory = evidence.get('trajectory') or {}
    loop = evidence.get('loop_alignment') or {}
    endpoint = evidence.get('public_loop_endpoint') or {}
    recovery = evidence.get('recovery') or {}
    lines = [
        '# MID-360 Continuous RKO-LIO Relocalization Gate',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- completion_ready: `{report.get('completion_ready')}`",
        f"- scope: `{report.get('scope', '')}`",
        f"- rko_poses: `{trajectory.get('poses', 0)}`",
        f"- trajectory_duration_sec: `{_fmt_float(trajectory.get('duration_sec'))}`",
        f"- relocalization_events: `{recovery.get('relocalization_events', 0)}`",
        f"- loop_candidates: `{loop.get('loop_candidates', 0)}`",
        f"- nearest_revisit_distance_m: `{_fmt_float(loop.get('nearest_revisit_distance_m'))}`",
        f"- public_endpoint_distance_m: `{_fmt_float(endpoint.get('endpoint_distance_m'))}`",
        '',
        '## Checks',
        '',
    ]
    for check in report.get('checks') or []:
        lines.append(
            f"- `{check.get('status', '')}` `{check.get('id', '')}`: "
            f"{check.get('message', '')}"
        )
    actions = report.get('next_actions') or []
    lines.extend(['', '## Next Actions', ''])
    if actions:
        lines.extend(f'- {action}' for action in actions)
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _checks(
    *,
    trajectory: dict[str, Any],
    autoware_verify: dict[str, Any],
    loop_alignment: dict[str, Any],
    public_endpoint: dict[str, Any],
    recovery: dict[str, Any],
    config: dict[str, Any],
    min_rko_poses: int,
    min_trajectory_duration_sec: float,
    min_relocalization_events: int,
    max_loop_distance_m: float,
    max_public_endpoint_distance_m: float,
    max_public_endpoint_stamp_error_sec: float,
) -> list[dict[str, str]]:
    loop_evidence = _loop_alignment_evidence(loop_alignment)
    return [
        _check(
            'continuous_rko_trajectory_complete',
            int(trajectory.get('poses') or 0) >= min_rko_poses
            and float(trajectory.get('duration_sec') or 0.0) >= min_trajectory_duration_sec,
            (
                f"poses={trajectory.get('poses', 0)} min={min_rko_poses}, "
                f"duration={_fmt_float(trajectory.get('duration_sec'))}s "
                f"min={_fmt_float(min_trajectory_duration_sec)}s"
            ),
        ),
        _check(
            'autoware_map_verify_pass',
            autoware_verify.get('status') == 'PASS',
            _verify_message(autoware_verify),
        ),
        _check(
            'loop_alignment_pass',
            loop_alignment.get('status') == 'PASS'
            and int(loop_evidence.get('loop_candidates') or 0) > 0
            and float(loop_evidence.get('max_loop_distance_m') or 999.0) <= max_loop_distance_m,
            _loop_message(loop_evidence, max_loop_distance_m),
        ),
        _check(
            'public_loop_endpoint_relocalized',
            bool(public_endpoint.get('has_start_pose'))
            and bool(public_endpoint.get('has_end_pose'))
            and _as_float(public_endpoint.get('start_stamp_error_sec'), 999.0)
            <= max_public_endpoint_stamp_error_sec
            and _as_float(public_endpoint.get('end_stamp_error_sec'), 999.0)
            <= max_public_endpoint_stamp_error_sec
            and _as_float(public_endpoint.get('endpoint_distance_m'), 999.0)
            <= max_public_endpoint_distance_m,
            _public_endpoint_message(
                public_endpoint,
                max_public_endpoint_distance_m,
                max_public_endpoint_stamp_error_sec,
            ),
        ),
        _check(
            'kidnap_relocalization_event_present',
            int(recovery.get('relocalization_events') or 0) >= min_relocalization_events
            and int(recovery.get('recovery_accept_events') or 0) >= min_relocalization_events,
            _recovery_message(recovery, min_relocalization_events),
        ),
        _check(
            'offline_node_completed',
            bool(recovery.get('offline_completed')),
            f"offline_completed={recovery.get('offline_completed')}",
        ),
        _check(
            'tracked_kidnap_config_matches_run',
            bool(config.get('matches_tracked_config'))
            and bool((config.get('run_config') or {}).get('enable_kidnap_relocalization'))
            and bool((config.get('run_config') or {}).get('reset_on_registration_failure')),
            _config_message(config),
        ),
    ]


def _trajectory_summary(run_dir: Path) -> dict[str, Any]:
    tum_path, lines = _longest_tum(run_dir)
    if not tum_path:
        return {'trajectory_path': '', 'poses': 0, 'duration_sec': 0.0}
    stamps = []
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        try:
            stamps.append(float(parts[0]))
        except ValueError:
            continue
    duration = max(stamps) - min(stamps) if len(stamps) >= 2 else 0.0
    return {
        'trajectory_path': str(tum_path),
        'poses': len(lines),
        'start_stamp': min(stamps) if stamps else None,
        'end_stamp': max(stamps) if stamps else None,
        'duration_sec': duration,
    }


def _public_loop_endpoint_summary(
    run_dir: Path,
    *,
    start_stamp_sec: float,
    end_stamp_sec: float,
) -> dict[str, Any]:
    tum_path, lines = _longest_tum(run_dir)
    poses = _parse_tum_poses(lines)
    summary: dict[str, Any] = {
        'trajectory_path': str(tum_path or ''),
        'target_start_stamp_sec': start_stamp_sec,
        'target_end_stamp_sec': end_stamp_sec,
        'has_start_pose': False,
        'has_end_pose': False,
        'start_stamp_error_sec': None,
        'end_stamp_error_sec': None,
        'endpoint_distance_m': None,
    }
    if not poses:
        return summary
    start_pose = min(poses, key=lambda pose: abs(pose['stamp'] - start_stamp_sec))
    end_pose = min(poses, key=lambda pose: abs(pose['stamp'] - end_stamp_sec))
    summary.update(
        {
            'has_start_pose': True,
            'has_end_pose': True,
            'start_pose': start_pose,
            'end_pose': end_pose,
            'start_stamp_error_sec': abs(start_pose['stamp'] - start_stamp_sec),
            'end_stamp_error_sec': abs(end_pose['stamp'] - end_stamp_sec),
            'endpoint_distance_m': _distance_xyz(start_pose, end_pose),
        }
    )
    return summary


def _parse_tum_poses(lines: list[str]) -> list[dict[str, float]]:
    poses: list[dict[str, float]] = []
    for line in lines:
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            poses.append(
                {
                    'stamp': float(parts[0]),
                    'x': float(parts[1]),
                    'y': float(parts[2]),
                    'z': float(parts[3]),
                }
            )
        except ValueError:
            continue
    return poses


def _distance_xyz(start_pose: dict[str, float], end_pose: dict[str, float]) -> float:
    dx = float(start_pose['x']) - float(end_pose['x'])
    dy = float(start_pose['y']) - float(end_pose['y'])
    dz = float(start_pose['z']) - float(end_pose['z'])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _longest_tum(run_dir: Path) -> tuple[Path | None, list[str]]:
    candidates = [
        path for pattern in ('*_tum_*.txt', '*.tum', 'traj_*.txt')
        for path in run_dir.rglob(pattern)
        if path.is_file()
    ]
    if not candidates:
        return None, []
    scored = [(path, _trajectory_lines(path)) for path in candidates]
    return max(scored, key=lambda item: len(item[1]))


def _trajectory_lines(path: Path) -> list[str]:
    return [
        line for line in path.read_text(encoding='utf-8', errors='replace').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    ]


def _verify_autoware_map(run_dir: Path) -> dict[str, Any]:
    verifier = MapVerifier(str(run_dir), check_bounds=False, verbose=False)
    stream = io.StringIO()
    with contextlib.redirect_stdout(stream):
        passed = verifier.run()
    return {
        'status': 'PASS' if passed else 'FAIL',
        'run_dir': str(run_dir),
        'passes': len(verifier.passes),
        'warnings': len(verifier.warnings),
        'failures': len(verifier.failures),
        'warning_messages': list(verifier.warnings),
        'failure_messages': list(verifier.failures),
        'stdout_tail': stream.getvalue().splitlines()[-20:],
    }


def _recovery_summary(log_path: Path) -> dict[str, Any]:
    text = log_path.read_text(encoding='utf-8', errors='replace') if log_path.is_file() else ''
    return {
        'log_path': str(log_path),
        'log_present': log_path.is_file(),
        'dropped_scan_events': text.count('Dropping scan during kidnap recovery'),
        'relocalization_events': text.count('Kidnap relocalization matched'),
        'recovery_accept_events': text.count('Kidnap recovery accepted scan'),
        'offline_completed': 'RKO LIO Offline Node took' in text,
        'last_num_submaps': _last_num_submaps(text),
    }


def _last_num_submaps(text: str) -> int:
    matches = re.findall(r'searching Loop, num_submaps:(\d+)', text)
    return int(matches[-1]) if matches else 0


def _config_summary(run_dir: Path, tracked_rko_config: Path) -> dict[str, Any]:
    run_config = _load_run_config(run_dir)
    tracked = _load_yaml(tracked_rko_config)
    keys = [
        'max_scan_delta_sec',
        'enable_kidnap_relocalization',
        'reset_on_registration_failure',
        'recovery_min_failures',
        'relocalize_after_scan_gap',
        'relocalization_min_correspondences',
        'relocalization_min_inlier_ratio',
        'relocalization_max_mean_error',
        'relocalization_max_correspondance_distance',
        'relocalization_yaw_samples',
        'relocalization_pose_stride',
        'relocalization_min_pose_separation',
        'relocalization_max_iterations',
    ]
    mismatches = [
        {
            'key': key,
            'run': run_config.get(key),
            'tracked': tracked.get(key),
        }
        for key in keys
        if run_config.get(key) != tracked.get(key)
    ]
    return {
        'run_config_path': str(_run_config_path(run_dir) or ''),
        'tracked_config_path': str(tracked_rko_config.expanduser().resolve()),
        'checked_keys': keys,
        'mismatches': mismatches,
        'matches_tracked_config': not mismatches,
        'run_config': {key: run_config.get(key) for key in keys},
        'tracked_config': {key: tracked.get(key) for key in keys},
    }


def _load_run_config(run_dir: Path) -> dict[str, Any]:
    path = _run_config_path(run_dir)
    if not path:
        return {}
    payload = _load_json(path)
    config = payload.get('config') if isinstance(payload, dict) else {}
    return config if isinstance(config, dict) else {}


def _run_config_path(run_dir: Path) -> Path | None:
    candidates = sorted(run_dir.rglob('config.json'))
    return candidates[0] if candidates else None


def _loop_alignment_evidence(loop_alignment: dict[str, Any]) -> dict[str, Any]:
    nearest = loop_alignment.get('nearest_revisit') or {}
    candidates = loop_alignment.get('loop_candidates') or []
    max_loop_distance = None
    if candidates:
        max_loop_distance = max(float(item.get('distance_m') or 0.0) for item in candidates)
    return {
        'status': loop_alignment.get('status'),
        'loop_candidates': len(candidates),
        'nearest_revisit_distance_m': nearest.get('distance_m'),
        'max_loop_distance_m': max_loop_distance,
        'trajectory': loop_alignment.get('trajectory') or {},
        'counts': loop_alignment.get('counts') or {},
    }


def _options_payload(options: ContinuousRelocalizationGateOptions) -> dict[str, Any]:
    payload = asdict(options)
    for key, value in list(payload.items()):
        if isinstance(value, Path):
            payload[key] = str(value.expanduser().resolve())
    return payload


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding='utf-8'))


def _load_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.expanduser().resolve().read_text(encoding='utf-8'))
    return payload if isinstance(payload, dict) else {}


def _check(check_id: str, ok: bool, message: str) -> dict[str, str]:
    return {'id': check_id, 'status': 'PASS' if ok else 'FAIL', 'message': message}


def _overall_status(checks: list[dict[str, str]]) -> str:
    return 'FAIL' if any(check.get('status') == 'FAIL' for check in checks) else 'PASS'


def _count_checks(checks: list[dict[str, str]]) -> dict[str, int]:
    return {
        'pass': sum(1 for check in checks if check.get('status') == 'PASS'),
        'fail': sum(1 for check in checks if check.get('status') == 'FAIL'),
        'total': len(checks),
    }


def _next_actions(checks: list[dict[str, str]]) -> list[str]:
    return [
        f"Fix `{check.get('id')}`: {check.get('message')}"
        for check in checks
        if check.get('status') == 'FAIL'
    ]


def _verify_message(verify: dict[str, Any]) -> str:
    return (
        f"status={verify.get('status')} passes={verify.get('passes', 0)} "
        f"warnings={verify.get('warnings', 0)} failures={verify.get('failures', 0)}"
    )


def _loop_message(loop: dict[str, Any], max_loop_distance_m: float) -> str:
    return (
        f"status={loop.get('status')} candidates={loop.get('loop_candidates', 0)} "
        f"nearest={_fmt_float(loop.get('nearest_revisit_distance_m'))}m "
        f"max={_fmt_float(loop.get('max_loop_distance_m'))}m threshold={max_loop_distance_m}m"
    )


def _public_endpoint_message(
    endpoint: dict[str, Any],
    max_endpoint_distance_m: float,
    max_stamp_error_sec: float,
) -> str:
    return (
        f"distance={_fmt_float(endpoint.get('endpoint_distance_m'))}m "
        f"threshold={_fmt_float(max_endpoint_distance_m)}m "
        f"start_dt={_fmt_float(endpoint.get('start_stamp_error_sec'))}s "
        f"end_dt={_fmt_float(endpoint.get('end_stamp_error_sec'))}s "
        f"stamp_threshold={_fmt_float(max_stamp_error_sec)}s"
    )


def _recovery_message(recovery: dict[str, Any], min_relocalization_events: int) -> str:
    return (
        f"relocalization={recovery.get('relocalization_events', 0)} "
        f"accepted={recovery.get('recovery_accept_events', 0)} "
        f"min={min_relocalization_events} dropped={recovery.get('dropped_scan_events', 0)}"
    )


def _config_message(config: dict[str, Any]) -> str:
    mismatches = config.get('mismatches') or []
    if not mismatches:
        return 'tracked recovery config matches run config'
    return 'mismatches=' + ', '.join(str(item.get('key')) for item in mismatches[:5])


def _fmt_float(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.3f}'
    except (TypeError, ValueError):
        return str(value)


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default
