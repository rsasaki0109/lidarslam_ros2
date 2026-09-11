"""Contracts for the ground-truth-free v41 GBA lifecycle audit."""

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit_v41_lifecycle',
    ROOT / 'scripts/audit_v41_gba_lifecycle_contract.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_worker_waits_before_request_and_success_requires_ack():
    result = MODULE.simulate_lifecycle([
        'worker_poll', 'request', 'worker_poll', 'worker_finish',
        'producer_poll', 'worker_poll'])
    assert result['trace'][0]['worker_exited'] is False
    assert result['final']['writeback'] is True
    assert result['final']['map_published'] is True
    assert result['final']['worker_exited'] is True


def test_resource_cancel_before_request_is_clean_and_has_no_writeback():
    result = MODULE.simulate_lifecycle([
        'worker_poll', 'cancel', 'worker_poll', 'producer_poll'])['final']
    assert result['cancelled'] is True
    assert result['worker_exited'] is True
    assert result['producer_done'] is True
    assert result['writeback'] is False
    assert result['map_published'] is False


def test_deadline_cancel_after_request_is_clean_and_has_no_writeback():
    result = MODULE.simulate_lifecycle([
        'request', 'worker_poll', 'cancel', 'worker_poll',
        'producer_poll'])['final']
    assert result['worker_ack'] is True
    assert result['producer_done'] is True
    assert result['writeback'] is False
    assert result['map_published'] is False


def test_unknown_or_invalid_transitions_fail_closed():
    for events in (['unknown'], ['worker_finish']):
        try:
            MODULE.simulate_lifecycle(events)
        except ValueError:
            pass
        else:
            raise AssertionError(f'unsafe lifecycle accepted: {events}')
