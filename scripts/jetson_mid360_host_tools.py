#!/usr/bin/env python3
"""Jetson host readiness helpers for MID-360 robot mapping."""

from __future__ import annotations

import json
import platform
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable


HOST_READINESS_JSON = 'jetson_mid360_host_readiness.json'
HOST_READINESS_MARKDOWN = 'jetson_mid360_host_readiness.md'


@dataclass(frozen=True)
class HostCheck:
    """Single host-readiness check."""

    id: str
    status: str
    message: str


@dataclass(frozen=True)
class HostReadinessOptions:
    """Operator-tunable host readiness thresholds."""

    output_dir: Path
    bag_dir: Path | None = None
    expected_bag_minutes: float = 10.0
    estimated_bag_mbps: float = 50.0
    bag_reserve_gb: float = 5.0
    min_bag_free_gb: float = 20.0
    min_output_free_gb: float = 5.0
    thermal_warn_c: float = 75.0
    thermal_fail_c: float = 85.0
    min_memory_available_gb: float = 1.0


class JetsonHostReadiness:
    """Inspect host-level prerequisites before a MID-360 robot run."""

    TOOL_SPECS = (
        ('ros2', 'fail', 'ROS 2 CLI is required; source the workspace/setup files.'),
        ('colcon', 'warn', 'colcon is useful for rebuilding on the Jetson.'),
        ('tegrastats', 'warn', 'tegrastats is useful for runtime thermal/load monitoring.'),
        ('nvpmodel', 'warn', 'nvpmodel is useful for confirming the power mode.'),
        ('jetson_clocks', 'warn', 'jetson_clocks is useful for repeatable field runs.'),
    )

    def __init__(
        self,
        host_root: Path = Path('/'),
        which: Callable[[str], str | None] | None = None,
        disk_usage: Callable[[Path], Any] | None = None,
        machine: Callable[[], str] | None = None,
    ) -> None:
        self._host_root = host_root
        self._which = which or shutil.which
        self._disk_usage = disk_usage or shutil.disk_usage
        self._machine = machine or platform.machine

    def build_report(self, options: HostReadinessOptions) -> dict[str, Any]:
        """Build a JSON-serializable host readiness report."""
        host = self._host_info()
        tools = self._tool_info()
        storage = self._storage_info(options)
        thermals = self._thermal_info(options)
        memory = self._memory_info(options)
        cpu_governors = self._cpu_governor_info()

        checks = []
        checks.extend(self._host_checks(host))
        checks.extend(self._tool_checks(tools))
        checks.extend(self._storage_checks(storage))
        checks.extend(self._thermal_checks(thermals, options))
        checks.extend(self._memory_checks(memory, options))
        checks.extend(self._cpu_governor_checks(cpu_governors))

        check_payload = [asdict(check) for check in checks]
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': self._status_from_checks(check_payload),
            'output_dir': str(options.output_dir),
            'options': self._options_payload(options),
            'host': host,
            'tools': tools,
            'storage': storage,
            'thermal': thermals,
            'memory': memory,
            'cpu_governors': cpu_governors,
            'checks': check_payload,
            'counts': self._count_checks(check_payload),
        }

    def write(self, report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
        """Write JSON and Markdown reports."""
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / HOST_READINESS_JSON
        markdown_path = output_dir / HOST_READINESS_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(self.render_markdown(report) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    @staticmethod
    def render_markdown(report: dict[str, Any]) -> str:
        """Render a concise operator-facing Markdown report."""
        lines = [
            '# Jetson MID-360 Host Readiness',
            '',
            f"- status: `{report['status']}`",
            f"- created_at: `{report['created_at']}`",
            f"- output_dir: `{report['output_dir']}`",
            '',
            '## Host',
            '',
            f"- model: `{report['host'].get('model') or 'unknown'}`",
            f"- arch: `{report['host'].get('arch') or 'unknown'}`",
            f"- l4t_release: `{report['host'].get('l4t_release') or 'unknown'}`",
            '',
            '## Checks',
            '',
        ]
        for check in report['checks']:
            lines.append(f"- `{check['status']}` `{check['id']}`: {check['message']}")

        lines.extend(['', '## Storage', ''])
        for target in report.get('storage', {}).get('targets', []):
            lines.append(
                f"- `{target['id']}` path `{target['path']}` free "
                f"`{target.get('free_gb')}` GiB required `{target.get('required_free_gb')}` GiB"
            )

        lines.extend(['', '## Tools', ''])
        for tool in report.get('tools', []):
            lines.append(
                f"- `{tool['name']}`: "
                f"`{'found' if tool['found'] else 'missing'}`"
            )
        return '\n'.join(lines)

    def _host_info(self) -> dict[str, Any]:
        return {
            'model': self._read_text('/proc/device-tree/model')
            or self._read_text('/sys/firmware/devicetree/base/model'),
            'arch': self._machine(),
            'l4t_release': self._read_text('/etc/nv_tegra_release'),
            'os_release': self._parse_os_release(),
        }

    def _tool_info(self) -> list[dict[str, Any]]:
        tools = []
        for name, severity, hint in self.TOOL_SPECS:
            path = self._which(name)
            tools.append({
                'name': name,
                'found': bool(path),
                'path': path or '',
                'missing_status': severity,
                'hint': hint,
            })
        return tools

    def _storage_info(self, options: HostReadinessOptions) -> dict[str, Any]:
        targets = [
            self._storage_target(
                target_id='output_dir',
                path=options.output_dir,
                required_free_gb=options.min_output_free_gb,
            )
        ]
        if options.bag_dir is not None:
            recording_gb = self._estimated_recording_gb(options)
            targets.append(
                self._storage_target(
                    target_id='bag_dir',
                    path=options.bag_dir,
                    required_free_gb=max(options.min_bag_free_gb, recording_gb),
                )
            )
        return {
            'expected_bag_minutes': options.expected_bag_minutes,
            'estimated_bag_mbps': options.estimated_bag_mbps,
            'bag_reserve_gb': options.bag_reserve_gb,
            'estimated_recording_gb': self._estimated_recording_gb(options),
            'targets': targets,
        }

    def _storage_target(
        self,
        target_id: str,
        path: Path,
        required_free_gb: float,
    ) -> dict[str, Any]:
        existing_path = self._nearest_existing_path(path)
        payload = {
            'id': target_id,
            'path': str(path),
            'checked_path': str(existing_path) if existing_path else '',
            'exists': path.exists(),
            'required_free_gb': round(required_free_gb, 3),
            'free_gb': None,
            'total_gb': None,
        }
        if existing_path is None:
            return payload

        usage = self._disk_usage(existing_path)
        payload['free_gb'] = round(float(usage.free) / 1024**3, 3)
        payload['total_gb'] = round(float(usage.total) / 1024**3, 3)
        return payload

    @staticmethod
    def _estimated_recording_gb(options: HostReadinessOptions) -> float:
        data_gb = (
            options.estimated_bag_mbps
            * options.expected_bag_minutes
            * 60.0
            / 1024.0
        )
        return round(data_gb + options.bag_reserve_gb, 3)

    @staticmethod
    def _nearest_existing_path(path: Path) -> Path | None:
        current = path
        while not current.exists():
            parent = current.parent
            if parent == current:
                return None
            current = parent
        return current

    def _thermal_info(self, options: HostReadinessOptions) -> dict[str, Any]:
        zones = []
        thermal_root = self._path('/sys/class/thermal')
        if thermal_root.is_dir():
            for zone_path in sorted(thermal_root.glob('thermal_zone*')):
                temp_c = self._thermal_temp_c(zone_path / 'temp')
                if temp_c is None:
                    continue
                zones.append({
                    'name': zone_path.name,
                    'type': self._read_file(zone_path / 'type') or zone_path.name,
                    'temp_c': round(temp_c, 2),
                })
        max_temp_c = max((zone['temp_c'] for zone in zones), default=None)
        return {
            'warn_c': options.thermal_warn_c,
            'fail_c': options.thermal_fail_c,
            'max_temp_c': max_temp_c,
            'zones': zones,
        }

    @staticmethod
    def _thermal_temp_c(path: Path) -> float | None:
        try:
            raw = path.read_text(encoding='utf-8').strip()
        except OSError:
            return None
        if not raw:
            return None
        value = float(raw)
        return value / 1000.0 if value > 200.0 else value

    def _memory_info(self, options: HostReadinessOptions) -> dict[str, Any]:
        meminfo = self._parse_meminfo()
        total_gb = self._kb_to_gib(meminfo.get('MemTotal'))
        available_gb = self._kb_to_gib(meminfo.get('MemAvailable'))
        return {
            'min_available_gb': options.min_memory_available_gb,
            'total_gb': total_gb,
            'available_gb': available_gb,
        }

    def _cpu_governor_info(self) -> dict[str, Any]:
        governors = {}
        cpu_root = self._path('/sys/devices/system/cpu')
        for path in sorted(cpu_root.glob('cpu*/cpufreq/scaling_governor')):
            cpu_name = path.parts[-3]
            governor = self._read_file(path)
            if governor:
                governors[cpu_name] = governor
        unique = sorted(set(governors.values()))
        return {
            'available': bool(governors),
            'governors': governors,
            'unique_governors': unique,
        }

    @staticmethod
    def _host_checks(host: dict[str, Any]) -> list[HostCheck]:
        checks = []
        model = str(host.get('model') or '')
        if model and ('jetson' in model.lower() or 'nvidia' in model.lower()):
            checks.append(HostCheck(
                'jetson_model', 'ok', f'Jetson/NVIDIA model detected: {model}',
            ))
        elif model:
            checks.append(HostCheck(
                'jetson_model', 'warn', f'Host model is not Jetson-specific: {model}',
            ))
        else:
            checks.append(HostCheck('jetson_model', 'warn', 'Jetson model file was not found.'))

        arch = str(host.get('arch') or '')
        checks.append(
            HostCheck(
                'host_arch',
                'ok' if arch == 'aarch64' else 'warn',
                f'Host architecture is {arch or "unknown"}.',
            )
        )
        return checks

    @staticmethod
    def _tool_checks(tools: list[dict[str, Any]]) -> list[HostCheck]:
        checks = []
        for tool in tools:
            if tool['found']:
                checks.append(HostCheck(
                    f"tool_{tool['name']}", 'ok',
                    f"{tool['name']} found at {tool['path']}",
                ))
            else:
                checks.append(
                    HostCheck(
                        f"tool_{tool['name']}",
                        tool['missing_status'],
                        f"{tool['name']} not found. {tool['hint']}",
                    )
                )
        return checks

    @staticmethod
    def _storage_checks(storage: dict[str, Any]) -> list[HostCheck]:
        checks = []
        for target in storage.get('targets', []):
            check_id = f"storage_{target['id']}"
            if target['id'] == 'bag_dir' and not target.get('exists'):
                checks.append(
                    HostCheck(
                        check_id,
                        'fail',
                        f"Bag storage path does not exist: {target['path']}.",
                    )
                )
                continue
            if target['checked_path'] == '':
                checks.append(
                    HostCheck(
                        check_id,
                        'fail',
                        f"No existing parent found for storage path {target['path']}.",
                    )
                )
                continue
            free_gb = target.get('free_gb')
            required_gb = float(target['required_free_gb'])
            if free_gb is None:
                checks.append(HostCheck(
                    check_id, 'fail',
                    f"Disk usage unavailable for {target['path']}.",
                ))
            elif float(free_gb) < required_gb:
                checks.append(
                    HostCheck(
                        check_id,
                        'fail',
                        f"{target['id']} has {free_gb} GiB free; require {required_gb:.3f} GiB.",
                    )
                )
            elif float(free_gb) < required_gb * 1.5:
                checks.append(
                    HostCheck(
                        check_id,
                        'warn',
                        f"{target['id']} has limited free space: {free_gb} GiB.",
                    )
                )
            else:
                checks.append(HostCheck(
                    check_id, 'ok',
                    f"{target['id']} free space is {free_gb} GiB.",
                ))
        return checks

    @staticmethod
    def _thermal_checks(
        thermals: dict[str, Any],
        options: HostReadinessOptions,
    ) -> list[HostCheck]:
        max_temp_c = thermals.get('max_temp_c')
        if max_temp_c is None:
            return [HostCheck(
                'thermal_zones', 'warn',
                'No thermal zone temperatures were readable.',
            )]
        if float(max_temp_c) >= options.thermal_fail_c:
            return [
                HostCheck(
                    'thermal_max_temp',
                    'fail',
                    f'Max thermal zone temperature is {max_temp_c} C.',
                )
            ]
        if float(max_temp_c) >= options.thermal_warn_c:
            return [
                HostCheck(
                    'thermal_max_temp',
                    'warn',
                    f'Max thermal zone temperature is {max_temp_c} C.',
                )
            ]
        return [HostCheck(
            'thermal_max_temp', 'ok',
            f'Max thermal zone temperature is {max_temp_c} C.',
        )]

    @staticmethod
    def _memory_checks(
        memory: dict[str, Any],
        options: HostReadinessOptions,
    ) -> list[HostCheck]:
        available_gb = memory.get('available_gb')
        if available_gb is None:
            return [HostCheck('memory_available', 'warn', 'MemAvailable was not readable.')]
        if float(available_gb) < options.min_memory_available_gb:
            return [
                HostCheck(
                    'memory_available',
                    'fail',
                    (
                        f'MemAvailable is {available_gb} GiB; '
                        f'require {options.min_memory_available_gb:.3f} GiB.'
                    ),
                )
            ]
        return [HostCheck('memory_available', 'ok', f'MemAvailable is {available_gb} GiB.')]

    @staticmethod
    def _cpu_governor_checks(cpu_governors: dict[str, Any]) -> list[HostCheck]:
        if not cpu_governors.get('available'):
            return [HostCheck('cpu_governor', 'warn', 'CPU governor files were not readable.')]
        unique = cpu_governors.get('unique_governors') or []
        if 'powersave' in unique:
            return [HostCheck(
                'cpu_governor', 'warn',
                f'CPU governors include powersave: {", ".join(unique)}',
            )]
        return [HostCheck('cpu_governor', 'ok', f'CPU governors: {", ".join(unique)}')]

    @staticmethod
    def _status_from_checks(checks: list[dict[str, Any]]) -> str:
        if any(check['status'] == 'fail' for check in checks):
            return 'FAIL'
        if any(check['status'] == 'warn' for check in checks):
            return 'WARN'
        return 'PASS'

    @staticmethod
    def _count_checks(checks: list[dict[str, Any]]) -> dict[str, int]:
        return {
            'ok': sum(1 for check in checks if check['status'] == 'ok'),
            'warn': sum(1 for check in checks if check['status'] == 'warn'),
            'fail': sum(1 for check in checks if check['status'] == 'fail'),
        }

    @staticmethod
    def _options_payload(options: HostReadinessOptions) -> dict[str, Any]:
        return {
            'bag_dir': str(options.bag_dir) if options.bag_dir else '',
            'expected_bag_minutes': options.expected_bag_minutes,
            'estimated_bag_mbps': options.estimated_bag_mbps,
            'bag_reserve_gb': options.bag_reserve_gb,
            'min_bag_free_gb': options.min_bag_free_gb,
            'min_output_free_gb': options.min_output_free_gb,
            'thermal_warn_c': options.thermal_warn_c,
            'thermal_fail_c': options.thermal_fail_c,
            'min_memory_available_gb': options.min_memory_available_gb,
        }

    def _parse_os_release(self) -> dict[str, str]:
        text = self._read_text('/etc/os-release')
        values = {}
        for line in text.splitlines():
            if '=' not in line:
                continue
            key, value = line.split('=', 1)
            values[key] = value.strip().strip('"')
        return values

    def _parse_meminfo(self) -> dict[str, int]:
        text = self._read_text('/proc/meminfo')
        values = {}
        for line in text.splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[0].endswith(':'):
                values[parts[0][:-1]] = int(parts[1])
        return values

    @staticmethod
    def _kb_to_gib(value: int | None) -> float | None:
        if value is None:
            return None
        return round(float(value) / 1024**2, 3)

    def _read_text(self, absolute_path: str) -> str:
        return self._read_file(self._path(absolute_path))

    @staticmethod
    def _read_file(path: Path) -> str:
        try:
            return path.read_text(encoding='utf-8').replace('\x00', '').strip()
        except OSError:
            return ''

    def _path(self, absolute_path: str) -> Path:
        return self._host_root / absolute_path.lstrip('/')


def payload_to_json(payload: dict[str, Any]) -> str:
    """Serialize report payloads consistently."""
    return json.dumps(payload, indent=2, sort_keys=True)


if __name__ == "__main__":
    raise SystemExit("jetson_mid360_host_tools is a library module; import it instead of running it directly.")
