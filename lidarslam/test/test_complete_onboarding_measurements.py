# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for SHA-bound onboarding measurement supplements."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shlex
import sys
from pathlib import Path  # noqa: I100

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'complete_onboarding_measurements.py'
SPEC = importlib.util.spec_from_file_location(
    'complete_onboarding_measurements', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
COMPLETE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COMPLETE)
CHECK = sys.modules['check_onboarding_trial']

TRIAL_PATH = (
    ROOT / 'docs' / 'evidence' / 'onboarding'
    / 'g0-source-humble-20260813-runtime-a.json'
)


def _write_incomplete_trial(path: Path) -> tuple[bytes, Path]:
    record = json.loads(TRIAL_PATH.read_text(encoding='utf-8'))
    record['measurements']['active_operator_time_sec'] = None
    record['measurements']['command_count'] = None
    receipt = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/first-map-validation-receipt-v1.schema.json'
        ),
        'status': 'PASS',
        'run': {
            'run_id': record['trial_id'],
            'product_version': record['environment']['product_version'],
            'git_commit': record['environment']['revision']['value'],
            'profile_id': 'rko_lio_graph_mid360_preset',
        },
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'autoware_status': 'PASS',
            'manifest_sha256': record['evidence']['manifest_sha256'],
        },
        'evidence': {
            'manifest': {
                'filename': 'run_manifest.json',
                'sha256': record['evidence']['manifest_sha256'],
            },
            'diagnosis': {
                'filename': 'autoware_map_diagnosis.json',
                'available': True,
                'sha256': '5' * 64,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': True,
                'sha256': '6' * 64,
            },
        },
        'checks': [
            {'id': f'check_{index}', 'passed': True, 'observed': 'pass'}
            for index in range(7)
        ],
        'shareability': {
            'contains_map_geometry': False,
            'contains_private_paths': False,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
    }
    receipt_raw = json.dumps(receipt, sort_keys=True).encode() + b'\n'
    record['evidence']['receipt_sha256'] = hashlib.sha256(
        receipt_raw
    ).hexdigest()
    raw = json.dumps(record, indent=2, sort_keys=True).encode('utf-8') + b'\n'
    path.write_bytes(raw)
    receipt_path = Path(f'{path}.receipt.json')
    receipt_path.write_bytes(receipt_raw)
    return raw, receipt_path


def test_completion_writes_supplement_and_makes_record_comparable(tmp_path):
    """Observed values produce a separate supplement and a comparable view."""
    record_path = tmp_path / 'trial.json'
    _, receipt_path = _write_incomplete_trial(record_path)
    supplement_path = tmp_path / 'trial.measurements.json'

    assert COMPLETE.main([
        str(record_path),
        '--output', str(supplement_path),
        '--validation-receipt', str(receipt_path),
        '--active-operator-time-sec', '31.5',
        '--command-count', '8',
        '--require-comparable',
        '--json',
    ]) == 0

    supplement = json.loads(supplement_path.read_text(encoding='utf-8'))
    assert supplement['trial_id'] == 'g0-source-humble-20260813-runtime-a'
    assert supplement['measurements']['active_operator_time_sec'] == 31.5
    assert supplement['measurement_sources']['command_count'] == (
        'operator-observation'
    )
    record, raw = COMPLETE._read_object(record_path)
    effective = COMPLETE.apply_measurement_supplement(
        record,
        supplement,
        record_bytes=raw,
    )
    assert COMPLETE.evaluate_trial(
        effective,
        require_evidence_binding=False,
    )['comparable'] is True
    assert CHECK.main([
        str(record_path),
        '--supplement', str(supplement_path),
        '--validation-receipt', str(receipt_path),
        '--json',
        '--require-comparable',
    ]) == 0
    updated = json.loads(record_path.read_text(encoding='utf-8'))
    assert updated['measurements']['command_count'] is None


def test_completion_uses_safe_default_output_and_json_next_command(
    tmp_path,
    capsys,
):
    """The common path needs no output-path bookkeeping and stays parseable."""
    record_path = tmp_path / 'trial.json'
    _, receipt_path = _write_incomplete_trial(record_path)
    default_output = Path(f'{record_path}.measurements.json')

    assert COMPLETE.main([
        str(record_path),
        '--validation-receipt', str(receipt_path),
        '--active-operator-time-sec', '31.5',
        '--command-count', '8',
        '--require-comparable',
        '--json',
    ]) == 0

    result = json.loads(capsys.readouterr().out)
    assert result['supplement_path'] == str(default_output)
    assert result['validation_command'] == shlex.join([
        'python3',
        'scripts/check_onboarding_trial.py',
        str(record_path),
        '--supplement',
        str(default_output),
        '--validation-receipt',
        str(receipt_path),
        '--json',
        '--require-comparable',
    ])
    assert default_output.is_file()


def test_prompted_json_keeps_prompts_off_stdout(tmp_path, monkeypatch, capsys):
    """Interactive observations do not corrupt a machine-readable result."""
    record_path = tmp_path / 'prompted.json'
    _, receipt_path = _write_incomplete_trial(record_path)
    answers = iter(['31.5', '8'])
    monkeypatch.setattr('builtins.input', lambda: next(answers))

    assert COMPLETE.main([
        str(record_path),
        '--validation-receipt', str(receipt_path),
        '--prompt-human-measurements',
        '--json',
    ]) == 0

    captured = capsys.readouterr()
    result = json.loads(captured.out)
    assert result['comparable'] is True
    assert 'Measurement card' in captured.err
    assert 'Record only observations from this exact trial' in captured.err
    assert 'Pause during downloads, builds, SLAM' in captured.err
    assert 'A pasted multiline' in captured.err
    assert 'unknown keeps the' in captured.err
    assert result['validation_command'] in captured.err
    assert 'Observed active operator seconds' in captured.err
    assert 'Observed human-submitted command count' in captured.err


def test_completion_rejects_overwrite_and_stale_base(tmp_path):
    """Stale hashes and attempts to replace known fields fail closed."""
    record_path = tmp_path / 'trial.json'
    _write_incomplete_trial(record_path)
    supplement_path = tmp_path / 'trial.measurements.json'

    assert COMPLETE.main([
        str(record_path),
        '--output', str(supplement_path),
        '--active-operator-time-sec', '31.5',
    ]) == 0
    supplement = json.loads(supplement_path.read_text(encoding='utf-8'))
    supplement['base_record_sha256'] = '0' * 64
    with pytest.raises(COMPLETE.TrialError, match='exact base record bytes'):
        record, raw = COMPLETE._read_object(record_path)
        COMPLETE.apply_measurement_supplement(
            record,
            supplement,
            record_bytes=raw,
        )

    known = json.loads(record_path.read_text(encoding='utf-8'))
    known['measurements']['active_operator_time_sec'] = 10.0
    known_raw = (
        json.dumps(known, indent=2, sort_keys=True).encode('utf-8') + b'\n'
    )
    known_path = tmp_path / 'known.json'
    known_path.write_bytes(known_raw)
    assert COMPLETE.main([
        str(known_path),
        '--output', str(tmp_path / 'known.measurements.json'),
        '--active-operator-time-sec', '11.0',
    ]) == 2
