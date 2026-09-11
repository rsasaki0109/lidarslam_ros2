#!/usr/bin/env python3
"""Run the MID-360 sample-session QA matrix."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_sample_session import (
    Mid360SampleSessionRunner,
    SampleSessionOptions,
)
from lidarslam_benchmark_tools.mid360_robot_sample_session_matrix_dashboard import MATRIX_HTML, write_matrix_dashboard
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


MATRIX_JSON = 'mid360_robot_sample_session_matrix.json'
MATRIX_MARKDOWN = 'mid360_robot_sample_session_matrix.md'
DEFAULT_MATRIX_SCENARIOS = ('pass', 'low-rate', 'missing-tf', 'frame-mismatch')
EXPECTED_SCENARIO_STATUS = {
    'pass': 'PASS',
    'low-rate': 'WARN',
    'missing-tf': 'WARN',
    'frame-mismatch': 'FAIL',
}


@dataclass(frozen=True)
class SampleSessionMatrixOptions:
    """Options for a sample-session QA matrix run."""

    profile_path: Path
    bag_root: Path
    output_dir: Path
    run_id_prefix: str = 'mid360_matrix'
    scenarios: tuple[str, ...] = DEFAULT_MATRIX_SCENARIOS
    duration_sec: float = 5.0
    pointcloud_rate_hz: float = 10.0
    imu_rate_hz: float = 100.0
    point_count: int = 32
    force: bool = False


class Mid360SampleSessionMatrixRunner:
    """Run sample sessions and summarize expected PASS/WARN/FAIL behavior."""

    def __init__(self, repo_root: Path) -> None:
        self._repo_root = repo_root
        self._session_runner = Mid360SampleSessionRunner(repo_root)

    def run(self, options: SampleSessionMatrixOptions) -> dict[str, Any]:
        output_dir = options.output_dir.expanduser().resolve()
        bag_root = options.bag_root.expanduser().resolve()
        profile_path = options.profile_path.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        bag_root.mkdir(parents=True, exist_ok=True)

        results = [self._run_scenario(options, profile_path, bag_root, output_dir, scenario)
                   for scenario in options.scenarios]
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if all(item['matched_expected'] for item in results) else 'FAIL',
            'profile_path': str(profile_path),
            'bag_root': str(bag_root),
            'output_dir': str(output_dir),
            'run_id_prefix': options.run_id_prefix,
            'matrix_json_path': str(output_dir / MATRIX_JSON),
            'matrix_markdown_path': str(output_dir / MATRIX_MARKDOWN),
            'matrix_html_path': str(output_dir / MATRIX_HTML),
            'scenarios': results,
            'counts': self._counts(results),
        }
        self.write_report(report, output_dir)
        return report

    def _run_scenario(
        self,
        options: SampleSessionMatrixOptions,
        profile_path: Path,
        bag_root: Path,
        output_dir: Path,
        scenario: str,
    ) -> dict[str, Any]:
        expected = EXPECTED_SCENARIO_STATUS.get(scenario, 'UNKNOWN')
        scenario_output_dir = output_dir / scenario
        run_id = f'{options.run_id_prefix}_{_safe_name(scenario)}'
        try:
            report = self._session_runner.run(
                SampleSessionOptions(
                    profile_path=profile_path,
                    bag_root=bag_root,
                    output_dir=scenario_output_dir,
                    run_id=run_id,
                    duration_sec=options.duration_sec,
                    pointcloud_rate_hz=options.pointcloud_rate_hz,
                    imu_rate_hz=options.imu_rate_hz,
                    point_count=options.point_count,
                    scenario=scenario,
                    force=options.force,
                )
            )
            observed = str(report.get('status', 'UNKNOWN')).upper()
            return {
                'scenario': scenario,
                'expected_status': expected,
                'observed_status': observed,
                'matched_expected': observed == expected,
                'run_id': report.get('run_id', run_id),
                'bag_path': report.get('bag_path', ''),
                'output_dir': str(scenario_output_dir),
                'field_session_json_path': report.get('field_session_json_path', ''),
                'readiness_json_path': report.get('readiness_json_path', ''),
                'recording_check_json_path': report.get('recording_check_json_path', ''),
                'map_plan_json_path': report.get('map_plan_json_path', ''),
                'dashboard_html_path': report.get('dashboard_html_path', ''),
                'error': '',
            }
        except Exception as exc:
            return {
                'scenario': scenario,
                'expected_status': expected,
                'observed_status': 'ERROR',
                'matched_expected': False,
                'run_id': run_id,
                'bag_path': str(bag_root / run_id),
                'output_dir': str(scenario_output_dir),
                'field_session_json_path': '',
                'readiness_json_path': '',
                'recording_check_json_path': '',
                'map_plan_json_path': '',
                'dashboard_html_path': '',
                'error': str(exc),
            }

    @staticmethod
    def write_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / MATRIX_JSON
        markdown_path = output_dir / MATRIX_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(render_matrix_markdown(report) + '\n', encoding='utf-8')
        html_path = write_matrix_dashboard(output_dir, report=report)
        return {'json': json_path, 'markdown': markdown_path, 'html': html_path}

    @staticmethod
    def _counts(results: list[dict[str, Any]]) -> dict[str, int]:
        return {
            'matched': sum(1 for item in results if item['matched_expected']),
            'mismatched': sum(1 for item in results if not item['matched_expected']),
            'total': len(results),
        }


def render_matrix_markdown(report: dict[str, Any]) -> str:
    """Render the sample-session matrix summary as Markdown."""
    lines = [
        '# MID-360 Robot Sample Session Matrix',
        '',
        f"- status: `{report['status']}`",
        f"- created_at: `{report['created_at']}`",
        f"- profile_path: `{report['profile_path']}`",
        f"- bag_root: `{report['bag_root']}`",
        f"- output_dir: `{report['output_dir']}`",
        f"- matrix_html: `{report.get('matrix_html_path', '')}`",
        '',
        '| Scenario | Expected | Observed | Match | Dashboard |',
        '| --- | --- | --- | --- | --- |',
    ]
    for item in report.get('scenarios', []):
        match = 'yes' if item.get('matched_expected') else 'no'
        dashboard = item.get('dashboard_html_path') or 'n/a'
        lines.append(
            f"| `{item.get('scenario')}` | `{item.get('expected_status')}` | "
            f"`{item.get('observed_status')}` | `{match}` | `{dashboard}` |"
        )
    errors = [item for item in report.get('scenarios', []) if item.get('error')]
    if errors:
        lines.extend(['', '## Errors', ''])
        for item in errors:
            lines.append(f"- `{item.get('scenario')}`: {item.get('error')}")
    return '\n'.join(lines)


def matrix_options_to_dict(options: SampleSessionMatrixOptions) -> dict[str, Any]:
    """Return options as JSON-serializable data."""
    data = asdict(options)
    data['profile_path'] = str(options.profile_path)
    data['bag_root'] = str(options.bag_root)
    data['output_dir'] = str(options.output_dir)
    data['scenarios'] = list(options.scenarios)
    return data


def _safe_name(value: str) -> str:
    return ''.join(ch if ch.isalnum() else '_' for ch in value).strip('_') or 'scenario'


if __name__ == "__main__":
    raise SystemExit("mid360_robot_sample_session_matrix is a library module; import it instead of running it directly.")
