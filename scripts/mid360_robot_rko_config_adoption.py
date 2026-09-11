#!/usr/bin/env python3
"""Adoption checks for tracked MID-360 RKO-LIO configs."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


RKO_CONFIG_ADOPTION_JSON = 'mid360_robot_rko_config_adoption.json'
RKO_CONFIG_ADOPTION_MARKDOWN = 'mid360_robot_rko_config_adoption.md'
RKO_CONFIG_PARAMETER_KEYS = (
    'voxel_size',
    'min_range',
    'max_range',
    'deskew',
    'double_downsample',
    'initialization_phase',
)


class RkoConfigAdoptionChecker:
    """Check whether a tracked config is backed by a quality-gated real-data case."""

    def __init__(
        self,
        quality_report_path: Path,
        config_path: Path,
        *,
        require_best: bool = False,
        tolerance: float = 1e-6,
    ) -> None:
        self._quality_report_path = quality_report_path.expanduser().resolve()
        self._config_path = config_path.expanduser().resolve()
        self._require_best = bool(require_best)
        self._tolerance = float(tolerance)

    def build_report(self) -> dict[str, Any]:
        """Build an adoption report."""
        quality_report = _load_json(self._quality_report_path)
        config = _load_yaml(self._config_path)
        config_parameters = _extract_config_parameters(config)
        cases = quality_report.get('cases') or []
        matched_case = _find_matching_case(
            config_parameters=config_parameters,
            cases=cases,
            tolerance=self._tolerance,
        )
        best_case = _best_gate_case(cases)
        checks = _build_checks(
            quality_report=quality_report,
            config_parameters=config_parameters,
            matched_case=matched_case,
            best_case=best_case,
            require_best=self._require_best,
        )
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if all(item['status'] == 'PASS' for item in checks) else 'FAIL',
            'quality_report_path': str(self._quality_report_path),
            'config_path': str(self._config_path),
            'require_best': self._require_best,
            'parameter_keys': list(RKO_CONFIG_PARAMETER_KEYS),
            'config_parameters': config_parameters,
            'matched_case': _case_summary(matched_case),
            'recommended_case': _case_summary(best_case),
            'parameter_diff_to_recommended': _parameter_diff(
                config_parameters,
                (best_case.get('parameters') or {}) if best_case else {},
                self._tolerance,
            ),
            'checks': checks,
        }


def write_rko_config_adoption_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON and Markdown adoption reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / RKO_CONFIG_ADOPTION_JSON
    markdown_path = output_dir / RKO_CONFIG_ADOPTION_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_rko_config_adoption_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_rko_config_adoption_markdown(report: dict[str, Any]) -> str:
    """Render the adoption report as Markdown."""
    matched = report.get('matched_case') or {}
    recommended = report.get('recommended_case') or {}
    lines = [
        '# MID-360 RKO-LIO Config Adoption',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- config_path: `{report.get('config_path', '')}`",
        f"- quality_report_path: `{report.get('quality_report_path', '')}`",
        f"- require_best: `{report.get('require_best')}`",
        f"- matched_case: `{matched.get('case_id', '')}`",
        f"- recommended_case: `{recommended.get('case_id', '')}`",
        '',
        '## Config Parameters',
        '',
    ]
    for key, value in (report.get('config_parameters') or {}).items():
        lines.append(f"- {key}: `{value}`")

    lines.extend(['', '## Checks', ''])
    for check in report.get('checks') or []:
        lines.append(
            f"- `{check.get('status', '')}` `{check.get('id', '')}`: "
            f"{check.get('message', '')}"
        )

    diffs = report.get('parameter_diff_to_recommended') or []
    lines.extend(['', '## Diff To Recommended', ''])
    if diffs:
        for item in diffs:
            lines.append(
                f"- `{item.get('key', '')}` config `{item.get('config')}` "
                f"recommended `{item.get('recommended')}`"
            )
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _extract_config_parameters(config: dict[str, Any]) -> dict[str, Any]:
    return {key: config.get(key) for key in RKO_CONFIG_PARAMETER_KEYS}


def _find_matching_case(
    config_parameters: dict[str, Any],
    cases: list[dict[str, Any]],
    tolerance: float,
) -> dict[str, Any]:
    matches = [
        row for row in cases
        if _parameters_match(config_parameters, row.get('parameters') or {}, tolerance)
    ]
    if not matches:
        return {}
    return sorted(matches, key=lambda row: int(row.get('rank') or 9999))[0]


def _best_gate_case(cases: list[dict[str, Any]]) -> dict[str, Any]:
    gate_pass = [
        row for row in cases
        if (row.get('quality_gate') or {}).get('status') == 'PASS'
    ]
    candidates = gate_pass or cases
    if not candidates:
        return {}
    return sorted(candidates, key=lambda row: int(row.get('rank') or 9999))[0]


def _parameters_match(
    config_parameters: dict[str, Any],
    case_parameters: dict[str, Any],
    tolerance: float,
) -> bool:
    return not _parameter_diff(config_parameters, case_parameters, tolerance)


def _parameter_diff(
    config_parameters: dict[str, Any],
    case_parameters: dict[str, Any],
    tolerance: float,
) -> list[dict[str, Any]]:
    diffs: list[dict[str, Any]] = []
    for key in RKO_CONFIG_PARAMETER_KEYS:
        config_value = config_parameters.get(key)
        case_value = case_parameters.get(key)
        if not _values_match(config_value, case_value, tolerance):
            diffs.append({'key': key, 'config': config_value, 'recommended': case_value})
    return diffs


def _values_match(left: Any, right: Any, tolerance: float) -> bool:
    if left is None or right is None:
        return left is right
    if isinstance(left, bool) or isinstance(right, bool):
        return bool(left) == bool(right)
    try:
        return abs(float(left) - float(right)) <= tolerance
    except Exception:
        return left == right


def _build_checks(
    quality_report: dict[str, Any],
    config_parameters: dict[str, Any],
    matched_case: dict[str, Any],
    best_case: dict[str, Any],
    require_best: bool,
) -> list[dict[str, str]]:
    missing_parameters = [
        key for key, value in config_parameters.items()
        if value is None
    ]
    checks = [
        _check(
            'quality_report_pass',
            quality_report.get('status') == 'PASS',
            f"quality_report_status={quality_report.get('status', '')}",
        ),
        _check(
            'config_parameters_present',
            not missing_parameters,
            (
                'missing=' + ','.join(missing_parameters)
                if missing_parameters
                else 'all required parameters present'
            ),
        ),
        _check(
            'recommended_case_present',
            bool(best_case),
            f"recommended_case={best_case.get('case_id', '') if best_case else ''}",
        ),
        _check(
            'config_matches_sweep_case',
            bool(matched_case),
            f"matched_case={matched_case.get('case_id', '') if matched_case else ''}",
        ),
        _check(
            'matched_case_gate_pass',
            (matched_case.get('quality_gate') or {}).get('status') == 'PASS',
            (
                f"matched_gate={_matched_gate_status(matched_case)}"
            ),
        ),
    ]
    if require_best:
        checks.append(
            _check(
                'matched_case_is_best',
                bool(matched_case)
                and bool(best_case)
                and matched_case.get('case_id') == best_case.get('case_id'),
                (
                    f"matched={matched_case.get('case_id', '') if matched_case else ''} "
                    f"best={best_case.get('case_id', '') if best_case else ''}"
                ),
            )
        )
    return checks


def _matched_gate_status(matched_case: dict[str, Any]) -> str:
    if not matched_case:
        return ''
    return (matched_case.get('quality_gate') or {}).get('status', '')


def _check(check_id: str, passed: bool, message: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'message': message,
    }


def _case_summary(row: dict[str, Any]) -> dict[str, Any]:
    if not row:
        return {}
    return {
        'case_id': row.get('case_id', ''),
        'rank': row.get('rank', 0),
        'status': row.get('status', ''),
        'gate_status': (row.get('quality_gate') or {}).get('status', ''),
        'quality_score': row.get('quality_score', 0),
        'parameters': row.get('parameters') or {},
        'output_dir': row.get('output_dir', ''),
    }


if __name__ == "__main__":
    raise SystemExit("mid360_robot_rko_config_adoption is a library module; import it instead of running it directly.")
