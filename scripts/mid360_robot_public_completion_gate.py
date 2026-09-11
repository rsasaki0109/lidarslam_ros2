#!/usr/bin/env python3
"""Completion gate for the public MID-360 segment-reset loop path."""

from __future__ import annotations

import contextlib
import io
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json
from lidarslam_benchmark_tools.verify_autoware_map import MapVerifier


PUBLIC_COMPLETION_GATE_JSON = 'mid360_robot_public_completion_gate.json'
PUBLIC_COMPLETION_GATE_MARKDOWN = 'mid360_robot_public_completion_gate.md'


@dataclass(frozen=True)
class PublicCompletionGateOptions:
    """Inputs for public MID-360 completion evidence."""

    repo_root: Path
    output_dir: Path
    loop_cloud_json: Path
    segment_reset_plan_json: Path
    start_run_dir: Path
    end_run_dir: Path
    segment_map_alignment_json: Path
    adoption_gate_json: Path
    dashboard_html: Path
    min_segment_rko_poses: int = 50
    scope: str = 'public_mid360_segment_reset_loop_completion'


class PublicCompletionGate:
    """Build a single PASS/FAIL report for public MID-360 completion criteria."""

    def build_report(self, options: PublicCompletionGateOptions) -> dict[str, Any]:
        """Build and write a completion report."""
        repo_root = options.repo_root.expanduser().resolve()
        output_dir = options.output_dir.expanduser().resolve()
        loop_cloud = _load_json(options.loop_cloud_json)
        segment_plan = _load_json(options.segment_reset_plan_json)
        alignment = _load_json(options.segment_map_alignment_json)
        adoption_gate = _load_json(options.adoption_gate_json)
        start_run = _run_summary(options.start_run_dir)
        end_run = _run_summary(options.end_run_dir)
        start_verify = verify_autoware_map(options.start_run_dir)
        end_verify = verify_autoware_map(options.end_run_dir)
        workflow = _workflow_summary(repo_root, options.dashboard_html)
        checks = _checks(
            loop_cloud=loop_cloud,
            segment_plan=segment_plan,
            alignment=alignment,
            adoption_gate=adoption_gate,
            start_run=start_run,
            end_run=end_run,
            start_verify=start_verify,
            end_verify=end_verify,
            workflow=workflow,
            min_segment_rko_poses=max(1, int(options.min_segment_rko_poses)),
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
                'loop_cloud_json': str(options.loop_cloud_json.expanduser().resolve()),
                'segment_reset_plan_json': str(options.segment_reset_plan_json.expanduser().resolve()),
                'start_run_dir': str(options.start_run_dir.expanduser().resolve()),
                'end_run_dir': str(options.end_run_dir.expanduser().resolve()),
                'segment_map_alignment_json': str(options.segment_map_alignment_json.expanduser().resolve()),
                'adoption_gate_json': str(options.adoption_gate_json.expanduser().resolve()),
                'dashboard_html': str(options.dashboard_html.expanduser().resolve()),
            },
            'evidence': {
                'loop_cloud': _loop_cloud_evidence(loop_cloud),
                'segment_reset_plan': _segment_plan_evidence(segment_plan),
                'start_segment_run': start_run,
                'end_segment_run': end_run,
                'start_autoware_map_verify': start_verify,
                'end_autoware_map_verify': end_verify,
                'segment_map_alignment': _alignment_evidence(alignment),
                'adoption_gate': _adoption_evidence(adoption_gate),
                'workflow': workflow,
            },
            'checks': checks,
            'counts': _count_checks(checks),
            'next_actions': _next_actions(checks),
        }
        write_public_completion_gate_report(report, output_dir)
        return report


def verify_autoware_map(run_dir: Path) -> dict[str, Any]:
    """Verify a run directory as an Autoware pointcloud map without printing."""
    run_dir = run_dir.expanduser().resolve()
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


def write_public_completion_gate_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON and Markdown completion-gate reports."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PUBLIC_COMPLETION_GATE_JSON
    markdown_path = output_dir / PUBLIC_COMPLETION_GATE_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_public_completion_gate_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_public_completion_gate_markdown(report: dict[str, Any]) -> str:
    """Render a concise Markdown completion report."""
    evidence = report.get('evidence') or {}
    alignment = evidence.get('segment_map_alignment') or {}
    adoption = evidence.get('adoption_gate') or {}
    lines = [
        '# MID-360 Public Completion Gate',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- completion_ready: `{report.get('completion_ready')}`",
        f"- scope: `{report.get('scope', '')}`",
        f"- aligned_median_nn_m: `{_fmt_float(alignment.get('aligned_median_nn_m'))}`",
        f"- aligned_p90_nn_m: `{_fmt_float(alignment.get('aligned_p90_nn_m'))}`",
        f"- adoption_matched_case: `{adoption.get('matched_case', '')}`",
        f"- adoption_recommended_case: `{adoption.get('recommended_case', '')}`",
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
    loop_cloud: dict[str, Any],
    segment_plan: dict[str, Any],
    alignment: dict[str, Any],
    adoption_gate: dict[str, Any],
    start_run: dict[str, Any],
    end_run: dict[str, Any],
    start_verify: dict[str, Any],
    end_verify: dict[str, Any],
    workflow: dict[str, Any],
    min_segment_rko_poses: int,
) -> list[dict[str, str]]:
    reset_pair = segment_plan.get('reset_pair') or {}
    start_endpoint = reset_pair.get('start') or {}
    end_endpoint = reset_pair.get('end') or {}
    adoption_decision = adoption_gate.get('decision') or {}
    workflow_entries = workflow.get('entrypoints') or {}
    return [
        _check('public_loop_cloud_pass', loop_cloud.get('status') == 'PASS', _status_msg(loop_cloud)),
        _check(
            'segment_reset_plan_pass',
            segment_plan.get('status') == 'PASS'
            and start_endpoint.get('status') == 'PASS'
            and end_endpoint.get('status') == 'PASS',
            _segment_pair_message(start_endpoint, end_endpoint),
        ),
        _check(
            'start_segment_rko_complete',
            start_run.get('map_saved') is True
            and int(start_run.get('rko_poses') or 0) >= min_segment_rko_poses,
            _run_message(start_run, min_segment_rko_poses),
        ),
        _check(
            'end_segment_rko_complete',
            end_run.get('map_saved') is True
            and int(end_run.get('rko_poses') or 0) >= min_segment_rko_poses,
            _run_message(end_run, min_segment_rko_poses),
        ),
        _check('start_autoware_map_verify_pass', start_verify.get('status') == 'PASS', _verify_message(start_verify)),
        _check('end_autoware_map_verify_pass', end_verify.get('status') == 'PASS', _verify_message(end_verify)),
        _check('segment_map_alignment_pass', alignment.get('status') == 'PASS', _alignment_message(alignment)),
        _check('rko_adoption_gate_pass', adoption_gate.get('status') == 'PASS', _adoption_message(adoption_gate)),
        _check(
            'tracked_config_matches_top_gate',
            adoption_decision.get('matched_case') == adoption_decision.get('recommended_case')
            and bool(adoption_decision.get('matched_case')),
            _adoption_message(adoption_gate),
        ),
        _check('dashboard_present', bool(workflow.get('dashboard_present')), f"dashboard={workflow.get('dashboard_html', '')}"),
        _check(
            'production_candidate_entrypoints_present',
            all(entry.get('exists') for entry in workflow_entries.values()),
            _entrypoint_message(workflow_entries),
        ),
    ]


def _run_summary(run_dir: Path) -> dict[str, Any]:
    run_dir = run_dir.expanduser().resolve()
    tum_path, poses = _longest_tum(run_dir)
    return {
        'run_dir': str(run_dir),
        'map_saved': (run_dir / 'map_projector_info.yaml').is_file()
        and (run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml').is_file(),
        'map_projector_info': str(run_dir / 'map_projector_info.yaml'),
        'pointcloud_map_metadata': str(run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'),
        'rko_tum_path': str(tum_path) if tum_path else '',
        'rko_poses': poses,
        'map_save_log': str(run_dir / 'map_save.log'),
    }


def _longest_tum(run_dir: Path) -> tuple[Path | None, int]:
    candidates = [
        path for pattern in ('*_tum_*.txt', '*.tum', 'traj_*.txt')
        for path in run_dir.rglob(pattern)
        if path.is_file()
    ]
    if not candidates:
        return None, 0
    scored = [(path, _line_count(path)) for path in candidates]
    return max(scored, key=lambda item: item[1])


def _workflow_summary(repo_root: Path, dashboard_html: Path) -> dict[str, Any]:
    entrypoints = {
        'production_candidate_session': repo_root / 'scripts' / 'run_mid360_robot_production_candidate_session.sh',
        'dashboard': repo_root / 'scripts' / 'generate_mid360_robot_session_dashboard.py',
        'bundle_export': repo_root / 'scripts' / 'export_mid360_robot_production_candidate_bundle.py',
        'bundle_import': repo_root / 'scripts' / 'import_mid360_robot_production_candidate_bundle.py',
        'release_gate': repo_root / 'scripts' / 'run_release_readiness_checks.sh',
        'public_completion_gate': repo_root / 'scripts' / 'run_mid360_robot_public_completion_gate.py',
    }
    return {
        'dashboard_html': str(dashboard_html.expanduser().resolve()),
        'dashboard_present': dashboard_html.expanduser().resolve().is_file(),
        'entrypoints': {
            key: {'path': str(path), 'exists': path.is_file()}
            for key, path in entrypoints.items()
        },
    }


def _loop_cloud_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    overlap = payload.get('overlap') or {}
    return {
        'status': payload.get('status', ''),
        'median_nn_m': overlap.get('symmetric_median_nn_m'),
        'p90_nn_m': overlap.get('symmetric_p90_nn_m'),
        'coverage_within_1m': overlap.get('coverage_within_1m'),
        'candidate_index': payload.get('candidate_index'),
    }


def _segment_plan_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    reset_pair = payload.get('reset_pair') or {}
    return {
        'status': payload.get('status', ''),
        'segment_count': payload.get('segment_count'),
        'gap_count': payload.get('gap_count'),
        'start_segment': ((reset_pair.get('start') or {}).get('segment') or {}).get('segment_id', ''),
        'end_segment': ((reset_pair.get('end') or {}).get('segment') or {}).get('segment_id', ''),
    }


def _alignment_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    aligned = payload.get('aligned_overlap') or {}
    return {
        'status': payload.get('status', ''),
        'aligned_median_nn_m': aligned.get('symmetric_median_nn_m'),
        'aligned_p90_nn_m': aligned.get('symmetric_p90_nn_m'),
        'coverage_within_1m': aligned.get('coverage_within_1m'),
    }


def _adoption_evidence(payload: dict[str, Any]) -> dict[str, Any]:
    decision = payload.get('decision') or {}
    return {
        'status': payload.get('status', ''),
        'quality_status': decision.get('quality_status', ''),
        'matched_case': decision.get('matched_case', ''),
        'recommended_case': decision.get('recommended_case', ''),
        'gate_pass_cases': decision.get('gate_pass_cases', 0),
    }


def _load_json(path: Path) -> dict[str, Any]:
    path = path.expanduser().resolve()
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _line_count(path: Path) -> int:
    try:
        return sum(1 for line in path.read_text(encoding='utf-8', errors='replace').splitlines() if line.strip())
    except Exception:
        return 0


def _check(check_id: str, passed: bool, message: str) -> dict[str, str]:
    return {'id': check_id, 'status': 'PASS' if passed else 'FAIL', 'message': message}


def _overall_status(checks: list[dict[str, str]]) -> str:
    return 'PASS' if all(check.get('status') == 'PASS' for check in checks) else 'FAIL'


def _count_checks(checks: list[dict[str, str]]) -> dict[str, int]:
    return {
        'pass': sum(1 for check in checks if check.get('status') == 'PASS'),
        'fail': sum(1 for check in checks if check.get('status') == 'FAIL'),
    }


def _next_actions(checks: list[dict[str, str]]) -> list[str]:
    failed = [check for check in checks if check.get('status') == 'FAIL']
    if not failed:
        return []
    return [f"Fix completion check `{check['id']}`: {check['message']}" for check in failed[:5]]


def _options_payload(options: PublicCompletionGateOptions) -> dict[str, Any]:
    payload = asdict(options)
    for key, value in list(payload.items()):
        if isinstance(value, Path):
            payload[key] = str(value)
    return payload


def _status_msg(payload: dict[str, Any]) -> str:
    return f"status={payload.get('status', 'MISSING')}"


def _segment_pair_message(start: dict[str, Any], end: dict[str, Any]) -> str:
    return (
        f"start={start.get('status', 'MISSING')}:{((start.get('segment') or {}).get('segment_id', ''))} "
        f"end={end.get('status', 'MISSING')}:{((end.get('segment') or {}).get('segment_id', ''))}"
    )


def _run_message(run: dict[str, Any], min_poses: int) -> str:
    return f"map_saved={run.get('map_saved')} rko_poses={run.get('rko_poses')} min={min_poses}"


def _verify_message(verify: dict[str, Any]) -> str:
    return (
        f"status={verify.get('status', '')} passes={verify.get('passes', 0)} "
        f"warnings={verify.get('warnings', 0)} failures={verify.get('failures', 0)}"
    )


def _alignment_message(alignment: dict[str, Any]) -> str:
    aligned = alignment.get('aligned_overlap') or {}
    return (
        f"status={alignment.get('status', '')} "
        f"median={_fmt_float(aligned.get('symmetric_median_nn_m'))} "
        f"p90={_fmt_float(aligned.get('symmetric_p90_nn_m'))}"
    )


def _adoption_message(adoption: dict[str, Any]) -> str:
    decision = adoption.get('decision') or {}
    return (
        f"status={adoption.get('status', '')} matched={decision.get('matched_case', '')} "
        f"recommended={decision.get('recommended_case', '')} gate_pass={decision.get('gate_pass_cases', 0)}"
    )


def _entrypoint_message(entrypoints: dict[str, dict[str, Any]]) -> str:
    missing = [key for key, value in entrypoints.items() if not value.get('exists')]
    return 'missing=' + (','.join(missing) if missing else 'none')


def _fmt_float(value: Any) -> str:
    if isinstance(value, (int, float)):
        return f'{float(value):.3f}'
    return 'n/a'


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_completion_gate is a library module; import it instead of running it directly.")
