#!/usr/bin/env python3
"""One-command public MID-360 RKO-LIO adoption gate."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_public_rko_quality_report import (
    RkoQualityGateThresholds,
    RkoQualityReportBuilder,
    write_rko_quality_report,
)
from lidarslam_benchmark_tools.mid360_robot_public_rko_sweep import (
    RKO_SWEEP_JSON,
    RkoSweepBuilder,
    RkoSweepCase,
    RkoSweepOptions,
    RkoSweepRunOptions,
)
from lidarslam_benchmark_tools.mid360_robot_rko_config_adoption import (
    RkoConfigAdoptionChecker,
    write_rko_config_adoption_report,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


RKO_ADOPTION_GATE_JSON = 'mid360_robot_public_rko_adoption_gate.json'
RKO_ADOPTION_GATE_MARKDOWN = 'mid360_robot_public_rko_adoption_gate.md'


@dataclass(frozen=True)
class RkoAdoptionGateOptions:
    """Inputs for the public RKO-LIO adoption gate."""

    repo_root: Path
    output_dir: Path
    config_path: Path
    sweep_path: Path
    mode: str = 'existing'
    require_best: bool = True
    thresholds: RkoQualityGateThresholds = RkoQualityGateThresholds()
    bag_path: Path | None = None
    base_rko_param: Path | None = None
    lidarslam_param: Path | None = None
    lidar_topic: str = '/livox/points'
    imu_topic: str = '/livox/imu'
    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'
    cases: tuple[RkoSweepCase, ...] = ()
    limit: int = 0
    allow_existing_output: bool = False
    run_timeout_sec: int = 90
    save_timeout_secs: int = 60
    startup_timeout_secs: int = 30
    offline_quiet_log_secs: int = 0


class RkoAdoptionGateRunner:
    """Run the sweep-quality-adoption gate pipeline."""

    def __init__(self, options: RkoAdoptionGateOptions) -> None:
        self._options = options

    def run(self) -> dict[str, Any]:
        """Run the configured gate pipeline and return a machine-readable report."""
        output_dir = self._options.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        sweep_manifest, sweep_paths = self._sweep_step(output_dir)
        sweep_path = Path(str(sweep_paths.get('json') or self._options.sweep_path)).resolve()

        quality_report = RkoQualityReportBuilder(
            sweep_path=sweep_path,
            thresholds=self._options.thresholds,
        ).build_report()
        quality_paths = write_rko_quality_report(quality_report, output_dir)

        adoption_report = RkoConfigAdoptionChecker(
            quality_report_path=quality_paths['json'],
            config_path=self._options.config_path,
            require_best=self._options.require_best,
        ).build_report()
        adoption_paths = write_rko_config_adoption_report(adoption_report, output_dir)

        checks = _gate_checks(
            mode=self._options.mode,
            sweep_manifest=sweep_manifest,
            quality_report=quality_report,
            adoption_report=adoption_report,
        )
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if all(item['status'] == 'PASS' for item in checks) else 'FAIL',
            'mode': self._options.mode,
            'output_dir': str(output_dir),
            'config_path': str(self._options.config_path.expanduser().resolve()),
            'require_best': self._options.require_best,
            'artifacts': {
                'sweep_json': str(sweep_path),
                'sweep_markdown': str(sweep_paths.get('markdown') or ''),
                'quality_json': str(quality_paths['json']),
                'quality_markdown': str(quality_paths['markdown']),
                'quality_html': str(quality_paths['html']),
                'adoption_json': str(adoption_paths['json']),
                'adoption_markdown': str(adoption_paths['markdown']),
            },
            'decision': {
                'status': adoption_report.get('status', ''),
                'matched_case': (adoption_report.get('matched_case') or {}).get('case_id', ''),
                'recommended_case': (
                    adoption_report.get('recommended_case') or {}
                ).get('case_id', ''),
                'quality_status': quality_report.get('status', ''),
                'sweep_status': sweep_manifest.get('status', ''),
                'gate_pass_cases': (quality_report.get('counts') or {}).get('gate_pass', 0),
            },
            'steps': {
                'sweep': _sweep_summary(sweep_manifest, sweep_path, self._options.mode),
                'quality': _quality_summary(quality_report, quality_paths['json']),
                'adoption': _adoption_summary(adoption_report, adoption_paths['json']),
            },
            'checks': checks,
        }
        gate_paths = write_rko_adoption_gate_report(report, output_dir)
        report['artifacts']['gate_json'] = str(gate_paths['json'])
        report['artifacts']['gate_markdown'] = str(gate_paths['markdown'])
        gate_paths['json'].write_text(payload_to_json(report) + '\n', encoding='utf-8')
        gate_paths['markdown'].write_text(
            render_rko_adoption_gate_markdown(report) + '\n', encoding='utf-8',
        )
        return report

    def _sweep_step(self, output_dir: Path) -> tuple[dict[str, Any], dict[str, Path | str]]:
        if self._options.mode == 'existing':
            sweep_path = self._options.sweep_path.expanduser().resolve()
            return _load_json(sweep_path), {
                'json': sweep_path,
                'markdown': _sweep_markdown_path(sweep_path),
            }
        if self._options.mode != 'run':
            raise ValueError(f'unsupported adoption gate mode: {self._options.mode}')
        if not self._options.bag_path:
            raise ValueError('bag_path is required in run mode')
        if not self._options.base_rko_param:
            raise ValueError('base_rko_param is required in run mode')
        if not self._options.lidarslam_param:
            raise ValueError('lidarslam_param is required in run mode')

        sweep_options = RkoSweepOptions(
            repo_root=self._options.repo_root,
            bag_path=self._options.bag_path,
            output_dir=output_dir,
            base_rko_param=self._options.base_rko_param,
            lidarslam_param=self._options.lidarslam_param,
            lidar_topic=self._options.lidar_topic,
            imu_topic=self._options.imu_topic,
            base_frame=self._options.base_frame,
            lidar_frame=self._options.lidar_frame,
            imu_frame=self._options.imu_frame,
            save_timeout_secs=max(1, int(self._options.save_timeout_secs)),
            startup_timeout_secs=max(1, int(self._options.startup_timeout_secs)),
            offline_quiet_log_secs=max(0, int(self._options.offline_quiet_log_secs)),
            allow_existing_output=self._options.allow_existing_output,
            limit=max(0, int(self._options.limit)),
        )
        builder = RkoSweepBuilder(options=sweep_options, cases=self._options.cases)
        manifest = builder.build(
            run=True,
            run_options=RkoSweepRunOptions(timeout_sec=max(0, int(self._options.run_timeout_sec))),
        )
        return manifest, builder.write(manifest)


def write_rko_adoption_gate_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON and Markdown gate reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / RKO_ADOPTION_GATE_JSON
    markdown_path = output_dir / RKO_ADOPTION_GATE_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_rko_adoption_gate_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_rko_adoption_gate_markdown(report: dict[str, Any]) -> str:
    """Render the gate report as Markdown."""
    decision = report.get('decision') or {}
    artifacts = report.get('artifacts') or {}
    lines = [
        '# MID-360 Public RKO-LIO Adoption Gate',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- mode: `{report.get('mode', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- config_path: `{report.get('config_path', '')}`",
        f"- require_best: `{report.get('require_best')}`",
        f"- matched_case: `{decision.get('matched_case', '')}`",
        f"- recommended_case: `{decision.get('recommended_case', '')}`",
        f"- gate_pass_cases: `{decision.get('gate_pass_cases', 0)}`",
        '',
        '## Artifacts',
        '',
    ]
    for key in (
        'sweep_json',
        'quality_json',
        'quality_html',
        'adoption_json',
        'gate_json',
    ):
        lines.append(f"- {key}: `{artifacts.get(key, '')}`")

    lines.extend(['', '## Checks', ''])
    for check in report.get('checks') or []:
        lines.append(
            f"- `{check.get('status', '')}` `{check.get('id', '')}`: "
            f"{check.get('message', '')}"
        )
    return '\n'.join(lines)


def _gate_checks(
    mode: str,
    sweep_manifest: dict[str, Any],
    quality_report: dict[str, Any],
    adoption_report: dict[str, Any],
) -> list[dict[str, str]]:
    sweep_status = str(sweep_manifest.get('status') or '')
    quality_counts = quality_report.get('counts') or {}
    checks = [
        _check('sweep_manifest_present', bool(sweep_manifest), f'sweep_status={sweep_status}'),
        _check(
            'sweep_not_blocked',
            mode == 'existing' or sweep_status not in ('FAIL', 'BLOCKED'),
            f'sweep_status={sweep_status}',
        ),
        _check(
            'quality_report_pass',
            quality_report.get('status') == 'PASS',
            f"quality_status={quality_report.get('status', '')}",
        ),
        _check(
            'quality_gate_has_pass_case',
            int(quality_counts.get('gate_pass') or 0) > 0,
            f"gate_pass={quality_counts.get('gate_pass', 0)}",
        ),
        _check(
            'adoption_pass',
            adoption_report.get('status') == 'PASS',
            f"adoption_status={adoption_report.get('status', '')}",
        ),
    ]
    return checks


def _check(check_id: str, passed: bool, message: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'message': message,
    }


def _sweep_summary(manifest: dict[str, Any], path: Path, mode: str) -> dict[str, Any]:
    return {
        'mode': mode,
        'status': manifest.get('status', ''),
        'path': str(path),
        'counts': manifest.get('counts') or {},
    }


def _quality_summary(report: dict[str, Any], path: Path) -> dict[str, Any]:
    return {
        'status': report.get('status', ''),
        'path': str(path),
        'counts': report.get('counts') or {},
        'best_case': report.get('best_case') or {},
    }


def _adoption_summary(report: dict[str, Any], path: Path) -> dict[str, Any]:
    return {
        'status': report.get('status', ''),
        'path': str(path),
        'matched_case': report.get('matched_case') or {},
        'recommended_case': report.get('recommended_case') or {},
    }


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _sweep_markdown_path(sweep_path: Path) -> str:
    markdown_path = sweep_path.with_suffix('.md')
    if markdown_path.name == RKO_SWEEP_JSON.replace('.json', '.md') and markdown_path.is_file():
        return str(markdown_path)
    candidate = sweep_path.parent / RKO_SWEEP_JSON.replace('.json', '.md')
    return str(candidate) if candidate.is_file() else ''


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_rko_adoption_gate is a library module; import it instead of running it directly.")
