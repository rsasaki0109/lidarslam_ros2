#!/usr/bin/env python3
"""Production-readiness gate for Jetson MID-360 robot mapping."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PRODUCTION_READINESS_JSON = 'mid360_robot_production_readiness.json'
PRODUCTION_READINESS_MARKDOWN = 'mid360_robot_production_readiness.md'


@dataclass(frozen=True)
class ProductionReadinessThresholds:
    """Thresholds required before calling a robot deployment production ready."""

    min_bag_duration_sec: float = 600.0
    min_pointcloud_hz: float = 5.0
    min_imu_hz: float = 50.0
    allow_warnings: bool = False
    allow_public_bag: bool = False


@dataclass(frozen=True)
class ProductionReadinessInputs:
    """Artifact paths used by the production readiness gate."""

    host_readiness: Path
    recording_check: Path
    readiness: Path
    map_diagnosis: Path
    adoption_gate: Path
    output_dir: Path


class Mid360ProductionReadinessGate:
    """Build a production-readiness decision from robot and public-gate artifacts."""

    def __init__(
        self,
        inputs: ProductionReadinessInputs,
        thresholds: ProductionReadinessThresholds | None = None,
    ) -> None:
        self._inputs = inputs
        self._thresholds = thresholds or ProductionReadinessThresholds()

    def build_report(self) -> dict[str, Any]:
        """Build a production-readiness report."""
        artifacts = {
            'host_readiness': _load_json(self._inputs.host_readiness),
            'recording_check': _load_json(self._inputs.recording_check),
            'readiness': _load_json(self._inputs.readiness),
            'map_diagnosis': _load_json(self._inputs.map_diagnosis),
            'adoption_gate': _load_json(self._inputs.adoption_gate),
        }
        paths = {
            'host_readiness': str(self._inputs.host_readiness.expanduser().resolve()),
            'recording_check': str(self._inputs.recording_check.expanduser().resolve()),
            'readiness': str(self._inputs.readiness.expanduser().resolve()),
            'map_diagnosis': str(self._inputs.map_diagnosis.expanduser().resolve()),
            'adoption_gate': str(self._inputs.adoption_gate.expanduser().resolve()),
        }
        evidence = _evidence_summary(artifacts)
        checks = _build_checks(artifacts, paths, evidence, self._thresholds)
        status = _overall_status(checks)
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'production_ready': status == 'PASS',
            'output_dir': str(self._inputs.output_dir.expanduser().resolve()),
            'thresholds': asdict(self._thresholds),
            'artifact_paths': paths,
            'evidence': evidence,
            'checks': checks,
            'counts': _count_checks(checks),
            'next_actions': _next_actions(checks, paths),
        }


def write_production_readiness_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON and Markdown production-readiness reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PRODUCTION_READINESS_JSON
    markdown_path = output_dir / PRODUCTION_READINESS_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_production_readiness_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_production_readiness_markdown(report: dict[str, Any]) -> str:
    """Render a concise production-readiness report."""
    evidence = report.get('evidence') or {}
    thresholds = report.get('thresholds') or {}
    lines = [
        '# MID-360 Robot Production Readiness',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- production_ready: `{report.get('production_ready')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- output_dir: `{report.get('output_dir', '')}`",
        f"- min_bag_duration_sec: `{thresholds.get('min_bag_duration_sec')}`",
        '',
        '## Evidence',
        '',
        f"- bag_path: `{evidence.get('bag_path', '')}`",
        '- estimated_bag_duration_sec: '
        f"`{_fmt_float(evidence.get('estimated_bag_duration_sec'))}`",
        f"- pointcloud_hz: `{_fmt_float(evidence.get('pointcloud_hz'))}`",
        f"- imu_hz: `{_fmt_float(evidence.get('imu_hz'))}`",
        f"- map_status: `{evidence.get('map_status', '')}`",
        f"- map_verify_result: `{evidence.get('map_verify_result', '')}`",
        f"- adoption_matched_case: `{evidence.get('adoption_matched_case', '')}`",
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
        for action in actions:
            lines.append(f'- {action}')
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _build_checks(
    artifacts: dict[str, dict[str, Any]],
    paths: dict[str, str],
    evidence: dict[str, Any],
    thresholds: ProductionReadinessThresholds,
) -> list[dict[str, str]]:
    checks: list[dict[str, str]] = []
    artifact_keys = (
        'host_readiness', 'recording_check', 'readiness', 'map_diagnosis', 'adoption_gate',
    )
    for key in artifact_keys:
        checks.append(_check(
            f'{key}_present',
            bool(artifacts.get(key)),
            f'{key}: {paths.get(key, "")}',
        ))

    checks.extend([
        _status_check(
            'host_readiness_pass',
            artifacts.get('host_readiness') or {},
            thresholds.allow_warnings,
        ),
        _status_check(
            'recording_check_pass',
            artifacts.get('recording_check') or {},
            thresholds.allow_warnings,
        ),
        _status_check(
            'readiness_pass',
            artifacts.get('readiness') or {},
            thresholds.allow_warnings,
        ),
        _check(
            'ready_for_mid360_launch',
            bool((artifacts.get('readiness') or {}).get('ready_for_mid360_launch')),
            'ready_for_mid360_launch='
            f"{(artifacts.get('readiness') or {}).get('ready_for_mid360_launch')}",
        ),
        _check(
            'public_rko_adoption_gate_pass',
            (artifacts.get('adoption_gate') or {}).get('status') == 'PASS',
            f"adoption_gate_status={(artifacts.get('adoption_gate') or {}).get('status', '')}",
        ),
        _check(
            'map_run_verified',
            evidence.get('map_status') == 'success'
            and evidence.get('map_verify_result') == 'PASS',
            (
                f"map_status={evidence.get('map_status', '')} "
                f"verify={evidence.get('map_verify_result', '')}"
            ),
        ),
        _check(
            'real_robot_bag',
            bool(evidence.get('bag_path'))
            and (
                thresholds.allow_public_bag
                or not _looks_like_public_or_sample_bag(evidence.get('bag_path', ''))
            ),
            f"bag_path={evidence.get('bag_path', '')}",
        ),
        _check(
            'bag_duration',
            _maybe_float(evidence.get('estimated_bag_duration_sec')) is not None
            and float(evidence.get('estimated_bag_duration_sec') or 0.0)
            >= thresholds.min_bag_duration_sec,
            (
                f"estimated={_fmt_float(evidence.get('estimated_bag_duration_sec'))} "
                f"min={thresholds.min_bag_duration_sec}"
            ),
        ),
        _check(
            'pointcloud_rate',
            _maybe_float(evidence.get('pointcloud_hz')) is not None
            and float(evidence.get('pointcloud_hz') or 0.0) >= thresholds.min_pointcloud_hz,
            (
                f"pointcloud_hz={_fmt_float(evidence.get('pointcloud_hz'))} "
                f"min={thresholds.min_pointcloud_hz}"
            ),
        ),
        _check(
            'imu_rate',
            _maybe_float(evidence.get('imu_hz')) is not None
            and float(evidence.get('imu_hz') or 0.0) >= thresholds.min_imu_hz,
            f"imu_hz={_fmt_float(evidence.get('imu_hz'))} min={thresholds.min_imu_hz}",
        ),
        _check(
            'stable_expected_frames',
            bool(evidence.get('pointcloud_frame_ok')) and bool(evidence.get('imu_frame_ok')),
            (
                f"pointcloud_frame_ok={evidence.get('pointcloud_frame_ok')} "
                f"imu_frame_ok={evidence.get('imu_frame_ok')}"
            ),
        ),
    ])
    return checks


def _status_check(
    check_id: str,
    payload: dict[str, Any],
    allow_warnings: bool,
) -> dict[str, str]:
    status = str(payload.get('status') or '').upper()
    ok = status == 'PASS' or (allow_warnings and status == 'WARN')
    return _check(check_id, ok, f'status={status or "MISSING"}')


def _evidence_summary(artifacts: dict[str, dict[str, Any]]) -> dict[str, Any]:
    readiness = artifacts.get('readiness') or {}
    recording = artifacts.get('recording_check') or {}
    map_diagnosis = artifacts.get('map_diagnosis') or {}
    adoption = artifacts.get('adoption_gate') or {}
    diagnostics = readiness.get('bag_diagnostics') or {}
    topics = diagnostics.get('topics') or {}
    pointcloud = topics.get('pointcloud') or {}
    imu = topics.get('imu') or {}
    bag_path = readiness.get('bag_path') or recording.get('bag_path') or ''
    return {
        'bag_path': bag_path,
        'estimated_bag_duration_sec': _estimate_bag_duration_sec(pointcloud, imu),
        'pointcloud_hz': _maybe_float(pointcloud.get('metadata_rate_hz')),
        'imu_hz': _maybe_float(imu.get('metadata_rate_hz')),
        'pointcloud_messages': int(pointcloud.get('metadata_message_count') or 0),
        'imu_messages': int(imu.get('metadata_message_count') or 0),
        'pointcloud_frame_ok': _topic_frame_ok(pointcloud),
        'imu_frame_ok': _topic_frame_ok(imu),
        'map_status': map_diagnosis.get('status', ''),
        'map_verify_result': (map_diagnosis.get('verify') or {}).get('result', ''),
        'adoption_matched_case': (
            (adoption.get('decision') or {}).get('matched_case')
            or ((adoption.get('matched_case') or {}).get('case_id'))
            or ''
        ),
        'adoption_recommended_case': (
            (adoption.get('decision') or {}).get('recommended_case')
            or ((adoption.get('recommended_case') or {}).get('case_id'))
            or ''
        ),
    }


def _topic_frame_ok(topic: dict[str, Any]) -> bool:
    if topic.get('stable_frame_id') is False:
        return False
    if topic.get('matches_expected_frame') is False:
        return False
    if topic.get('sampled_message_count') == 0:
        return False
    return True


def _estimate_bag_duration_sec(pointcloud: dict[str, Any], imu: dict[str, Any]) -> float | None:
    estimates = []
    for topic in (pointcloud, imu):
        count = _maybe_float(topic.get('metadata_message_count'))
        rate = _maybe_float(topic.get('metadata_rate_hz'))
        if count is not None and rate is not None and rate > 0.0:
            estimates.append(count / rate)
    if not estimates:
        return None
    return max(estimates)


def _looks_like_public_or_sample_bag(path: str) -> bool:
    text = path.replace('\\', '/').lower()
    blocked_tokens = (
        '/datasets/mid360_public/',
        '/datasets/mid360_public_segments/',
        'mid360_sample',
        'sample_session',
        'synthetic',
    )
    return any(token in text for token in blocked_tokens)


def _check(check_id: str, passed: bool, message: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'message': message,
    }


def _overall_status(checks: list[dict[str, str]]) -> str:
    if any(check['status'] == 'FAIL' for check in checks):
        return 'FAIL'
    if any(check['status'] == 'WARN' for check in checks):
        return 'WARN'
    return 'PASS'


def _count_checks(checks: list[dict[str, str]]) -> dict[str, int]:
    return {
        'pass': sum(1 for check in checks if check['status'] == 'PASS'),
        'warn': sum(1 for check in checks if check['status'] == 'WARN'),
        'fail': sum(1 for check in checks if check['status'] == 'FAIL'),
    }


def _next_actions(checks: list[dict[str, str]], paths: dict[str, str]) -> list[str]:
    failed = {check['id'] for check in checks if check['status'] == 'FAIL'}
    actions = []
    if 'host_readiness_present' in failed or 'host_readiness_pass' in failed:
        actions.append(
            'Run check_jetson_mid360_host_readiness.py on the Jetson '
            'with the production bag directory.'
        )
    if 'recording_check_present' in failed or 'recording_check_pass' in failed:
        actions.append(
            'Run check_mid360_robot_recording.sh on a real robot recording '
            'with the final robot profile.'
        )
    if (
        'readiness_present' in failed
        or 'readiness_pass' in failed
        or 'ready_for_mid360_launch' in failed
    ):
        actions.append(
            'Run check_mid360_robot_readiness.py with the exact production '
            'robot profile and bag.'
        )
    if 'map_diagnosis_present' in failed or 'map_run_verified' in failed:
        actions.append(
            'Run the production bag through mapping, call /map_save, then run '
            'diagnose_autoware_map_run.py --write.'
        )
    if 'adoption_gate_present' in failed or 'public_rko_adoption_gate_pass' in failed:
        actions.append(
            'Run run_mid360_robot_public_rko_adoption_gate.py from the latest '
            'public MID-360 sweep evidence.'
        )
    if 'real_robot_bag' in failed:
        actions.append(
            'Use an actual robot field bag, not public dataset or synthetic sample evidence.'
        )
    if 'bag_duration' in failed:
        actions.append(
            'Record a longer stationary/walking production candidate bag and rerun readiness.'
        )
    if 'stable_expected_frames' in failed:
        actions.append('Fix frame IDs and static TF/profile measurements before mapping.')
    return actions[:8]


def _load_json(path: Path) -> dict[str, Any]:
    path = path.expanduser()
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _maybe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _fmt_float(value: Any) -> str:
    number = _maybe_float(value)
    return '' if number is None else f'{number:.3f}'


if __name__ == "__main__":
    raise SystemExit("mid360_robot_production_readiness is a library module; import it instead of running it directly.")
