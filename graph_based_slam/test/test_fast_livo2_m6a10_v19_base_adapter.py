# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""v19 typed base-adapter and injected lifecycle gates."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fast_livo2_m6a10_v19_base_adapter as ADAPTER  # noqa: E402,I100
import run_fast_livo2_m6a10_v19_formal as LAUNCH  # noqa: E402,I100


def test_actual_v12_surface_is_allowlisted_and_has_no_missing_contract_symbol():
    surface = ADAPTER.load_base_surface()
    assert surface.module.__file__ == str(ADAPTER.BASE_MODULE_PATH)
    assert set(surface.as_identity()['required_symbols']) == set(ADAPTER.REQUIRED_SYMBOLS)
    assert 'CONTRACT_VERSION' not in vars(surface.module)
    assert surface.phase_contract.startswith('m6a10-online-compute-')
    assert surface.transport_contract.startswith('m6a10-v12-')
    assert (
        surface.module_sha256 == hashlib.sha256(ADAPTER.BASE_MODULE_PATH.read_bytes()).hexdigest()
    )


def test_adapter_rejects_missing_and_forbidden_symbols():
    missing = types.ModuleType('missing_v12')
    with pytest.raises(ADAPTER.BaseAdapterError, match='missing v12 symbols'):
        ADAPTER.load_base_surface(missing)

    forbidden = types.ModuleType('forbidden_v12')
    for name in ADAPTER.REQUIRED_SYMBOLS:
        setattr(forbidden, name, (lambda: None) if name in ADAPTER.CALLABLE_SYMBOLS else 'value')
    forbidden.CONTRACT_VERSION = 'must-not-be-used'
    with pytest.raises(ADAPTER.BaseAdapterError, match='forbidden v12 symbols'):
        ADAPTER.load_base_surface(forbidden)


def test_ast_gate_rejects_direct_base_module_access(tmp_path):
    ADAPTER.assert_launcher_uses_adapter(ROOT / 'scripts/run_fast_livo2_m6a10_v19_formal.py')
    bad = tmp_path / 'bad_launcher.py'
    bad.write_text(
        'import run_fast_livo2_m6a10_v12_formal as v12\nvalue = v12.PHASE_CONTRACT\n',
        encoding='utf-8',
    )
    with pytest.raises(ADAPTER.BaseAdapterError, match='direct base-module'):
        ADAPTER.assert_launcher_uses_adapter(bad)

    bad_contract = tmp_path / 'bad_contract.py'
    bad_contract.write_text('value = CONTRACT_VERSION\n', encoding='utf-8')
    with pytest.raises(ADAPTER.BaseAdapterError, match='CONTRACT_VERSION'):
        ADAPTER.assert_launcher_uses_adapter(bad_contract)


def test_v18b_authorized_fixture_and_pre_popen_closure_are_bound():
    value = ADAPTER.verify_v18b_lineage(
        authorization_path=LAUNCH.V18B_AUTHORIZATION_PATH,
        authorization_sha256=LAUNCH.V18B_AUTHORIZATION_SHA256,
        authorization_sidecar_sha256=LAUNCH.V18B_AUTHORIZATION_SIDECAR_SHA256,
        closure_path=LAUNCH.V18B_CLOSURE_PATH,
        closure_sha256=LAUNCH.V18B_CLOSURE_SHA256,
        closure_sidecar_sha256=LAUNCH.V18B_CLOSURE_SIDECAR_SHA256,
        expected_attempt_root=LAUNCH.V18B_ATTEMPT_ROOT,
    )
    assert value['authorization']['status'] == 'AUTHORIZED'
    assert value['closure']['failure_kind'] == 'V18_FAIL_CLOSED'
    assert value['closure']['popen_count'] == 0


class _FakeProcess:
    pid = 19019

    def __init__(self, events):
        self.events = events

    def wait(self):
        self.events.append('wait')
        return 0


class _FakeMonitor:
    def __init__(self, events):
        self.events = events

    def start(self):
        self.events.append('monitor_start')

    def allow_owned_pid(self, pid):
        self.events.append(('allow_owned_pid', pid))

    def stop(self):
        self.events.append('monitor_stop')

    def finalize(self, _path):
        self.events.append('monitor_finalize')
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
        }


def test_v19_main_imports_actual_base_and_seals_pass_with_one_fake_popen(tmp_path, capsys):
    events = []
    surface = ADAPTER.load_base_surface()

    def authorize(_config):
        lineage = ADAPTER.verify_v18b_lineage(
            authorization_path=LAUNCH.V18B_AUTHORIZATION_PATH,
            authorization_sha256=LAUNCH.V18B_AUTHORIZATION_SHA256,
            authorization_sidecar_sha256=LAUNCH.V18B_AUTHORIZATION_SIDECAR_SHA256,
            closure_path=LAUNCH.V18B_CLOSURE_PATH,
            closure_sha256=LAUNCH.V18B_CLOSURE_SHA256,
            closure_sidecar_sha256=LAUNCH.V18B_CLOSURE_SIDECAR_SHA256,
            expected_attempt_root=LAUNCH.V18B_ATTEMPT_ROOT,
        )
        return {
            'authorized': True,
            'formal_execution': True,
            'status': 'AUTHORIZED',
            'lineage': lineage,
        }

    def process(argv, cwd):
        events.append(('popen', list(argv), cwd))
        return _FakeProcess(events)

    def capture(_config, _root):
        events.append('capture')
        return {'callback': {'status': 'PASS'}, 'terminal': {'status': 'PASS'}}

    def compose(raw, _config):
        events.append('compose')
        assert raw['terminal']['status'] == 'PASS'
        return {'status': 'PASS', 'contract': LAUNCH.CLOSURE_CONTRACT}

    root = tmp_path / 'fresh'
    code = LAUNCH.main(
        [
            '--root',
            str(root),
            '--authorization',
            str(LAUNCH.V18B_AUTHORIZATION_PATH),
            '--authorization-sha256',
            LAUNCH.V18B_AUTHORIZATION_SHA256,
        ],
        runtime={
            'authorization_validator': authorize,
            'identity_probe': lambda _config: {'image_id': surface.image_id, 'opened': False},
            'bag_probe': lambda _config: {'path': LAUNCH.INPUT_PATH, 'opened': False},
            'process_factory': process,
            'monitor_factory': lambda _root: _FakeMonitor(events),
            'argv_builder': lambda _config, output, _surface: ['fake-runner', str(output)],
            'raw_capture': capture,
            'composer': compose,
            'now': '2026-08-24T00:00:00+00:00',
        },
    )
    assert code == 0
    assert json.loads(capsys.readouterr().out)['status'] == 'PASS'
    assert len([item for item in events if isinstance(item, tuple) and item[0] == 'popen']) == 1
    assert events.count('monitor_start') == 1
    assert events.count('monitor_finalize') == 1
    assert events.count('wait') == 1
    receipt = root / 'closure_receipt.json'
    assert receipt.stat().st_mode & 0o777 == 0o444
    value = json.loads(receipt.read_text(encoding='utf-8'))
    assert value['status'] == 'PASS'
    assert value['execution']['popen_count'] == 1
    assert value['base_surface']['module'] == 'run_fast_livo2_m6a10_v12_formal'
    digest = hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert (root / 'closure_receipt.json.sha256').read_bytes() == (
        digest + '  closure_receipt.json\n'
    ).encode('ascii')


def test_v19_default_is_unauthorized_and_does_not_start(tmp_path):
    root = tmp_path / 'unauthorized'
    result = LAUNCH.run_formal(LAUNCH.CandidateConfig(root=root))
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'V19_AUTHORIZATION_REQUIRED'
    assert result['execution']['popen_count'] == 0
