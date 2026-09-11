#!/usr/bin/env python3
"""Audit the v41 standalone-GBA lifecycle without reading benchmark truth.

The v40 worker could exit after ``finish`` but before top-down HBA published
its request.  The producer would then enter an unbounded busy-wait.  This
diagnostic checks the source-level repair and exercises a small deterministic
request/acknowledgement model for success, resource cancellation, deadline
cancellation, and shutdown.  It never launches the mapper or changes a map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
from typing import Any


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def simulate_lifecycle(events: list[str]) -> dict[str, Any]:
    state: dict[str, Any] = {
        'request_pending': False,
        'worker_active': False,
        'worker_ack': False,
        'worker_exited': False,
        'producer_done': False,
        'cancelled': False,
        'writeback': False,
        'map_published': False,
    }
    trace: list[dict[str, Any]] = []

    for event in events:
        if event == 'request':
            if state['producer_done'] or state['cancelled']:
                raise ValueError('request after terminal producer state')
            state['request_pending'] = True
        elif event == 'cancel':
            state['cancelled'] = True
        elif event == 'worker_poll':
            if state['worker_exited']:
                pass
            elif state['cancelled']:
                state['worker_active'] = False
                state['worker_ack'] = True
                state['worker_exited'] = True
            elif state['request_pending']:
                state['request_pending'] = False
                state['worker_active'] = True
            elif state['producer_done']:
                state['worker_exited'] = True
        elif event == 'worker_finish':
            if not state['worker_active']:
                raise ValueError('worker_finish without active request')
            state['worker_active'] = False
            state['worker_ack'] = True
        elif event == 'producer_poll':
            if state['cancelled'] and state['worker_ack']:
                state['producer_done'] = True
            elif state['worker_ack'] and not state['cancelled']:
                state['writeback'] = True
                state['map_published'] = True
                state['producer_done'] = True
        else:
            raise ValueError(f'unknown lifecycle event: {event}')
        trace.append({'event': event, **state})

    return {'events': events, 'trace': trace, 'final': state}


def source_contract(v40_text: str, v41_text: str) -> dict[str, bool]:
    def has(text: str, pattern: str) -> bool:
        return re.search(pattern, text, re.MULTILINE | re.DOTALL) is not None

    return {
        'v40_worker_may_exit_before_request': has(
            v40_text, r'is_finish\s*&&\s*gba_flag\s*==\s*0\)\s*break'),
        'v40_producer_has_unbounded_busy_wait': has(
            v40_text, r'while\(gba_flag\);'),
        'v41_uses_atomic_request_state': has(
            v41_text, r'atomic<int>\s+gba_flag\{0\}'),
        'v41_worker_exit_requires_producer_done': has(
            v41_text,
            r'gba_producer_done\.load\(\).*?gba_flag\.load\(\)\s*==\s*0'),
        'v41_wait_is_shutdown_aware_and_yields': has(
            v41_text,
            r'while\(gba_flag\.load\(\)\s*!=\s*0\s*&&\s*ros::ok\(\)\)'
            r'.*?ros::WallDuration\(0\.01\)\.sleep\(\)'),
        'v41_guard_defaults_off': has(
            v41_text,
            r'GBA/runtime_guard_enable"\s*,\s*'
            r'gba_runtime_guard_enabled\s*,\s*false'),
        'v41_diagnostic_defaults_off': has(
            v41_text,
            r'GBA/runtime_diagnostic_enable"\s*,\s*'
            r'gba_runtime_diagnostic_enabled\s*,\s*false'),
        'v41_has_rss_and_deadline_cancellation': (
            'cancel_rss_limit' in v41_text and
            'cancel_backend_deadline' in v41_text),
        'v41_cancel_returns_before_writeback': has(
            v41_text,
            r'cancelled_before_writeback.*?return false;.*?'
            r'set_state\(results\.at'),
        'v41_stage_markers_are_flushed': has(
            v41_text, r'V41_GBA stage=.*?fflush\(stdout\)'),
    }


def build_report(v40_path: Path, v41_path: Path) -> dict[str, Any]:
    source = source_contract(
        v40_path.read_text(encoding='utf-8', errors='replace'),
        v41_path.read_text(encoding='utf-8', errors='replace'))
    scenarios = {
        'success': simulate_lifecycle([
            'worker_poll', 'request', 'worker_poll', 'worker_finish',
            'producer_poll', 'worker_poll']),
        'rss_cancel_before_request': simulate_lifecycle([
            'worker_poll', 'cancel', 'worker_poll', 'producer_poll']),
        'deadline_cancel_after_request': simulate_lifecycle([
            'request', 'worker_poll', 'cancel', 'worker_poll',
            'producer_poll']),
    }
    success = scenarios['success']['final']
    cancelled = [
        scenarios['rss_cancel_before_request']['final'],
        scenarios['deadline_cancel_after_request']['final'],
    ]
    lifecycle_valid = (
        success['writeback'] and success['map_published'] and
        success['producer_done'] and success['worker_exited'] and
        all(item['producer_done'] and item['worker_exited'] and
            not item['writeback'] and not item['map_published']
            for item in cancelled)
    )
    go = all(source.values()) and lifecycle_valid
    return {
        'schema_version': 1,
        'status': 'ground_truth_free_v41_gba_lifecycle_audit',
        'accuracy_ground_truth_accessed': False,
        'trajectory_or_map_mutated': False,
        'decision': (
            'GO_V41_RUNTIME_ONLY_DIAGNOSTIC' if go
            else 'NO_GO_V41_LIFECYCLE_CONTRACT'),
        'inputs': {
            'v40_source': str(v40_path.resolve()),
            'v40_source_sha256': sha256(v40_path),
            'v41_source': str(v41_path.resolve()),
            'v41_source_sha256': sha256(v41_path),
        },
        'source_contract': source,
        'lifecycle_contract_valid': lifecycle_valid,
        'scenarios': scenarios,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--v40-source', required=True, type=Path)
    parser.add_argument('--v41-source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    report = build_report(args.v40_source, args.v41_source)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({
        'decision': report['decision'],
        'lifecycle_contract_valid': report['lifecycle_contract_valid'],
        'output': str(args.output.resolve()),
    }, indent=2, sort_keys=True))
    return 0 if report['decision'].startswith('GO_') else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
