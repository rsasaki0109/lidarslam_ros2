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

"""Focused fail-closed tests for FAST-LIVO2 v7 timeout/watchdog behavior."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import types

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = ROOT / 'scripts' / 'run_fast_livo2_benchmark.py'
FEEDER_PATH = ROOT / 'scripts' / 'fast_livo2_m6a10_feeder.py'
PROFILE_PATH = ROOT / 'configs' / 'slam_benchmark_profiles' / ('fast_livo2_m6a10_v7_formal.yaml')


def _load_runner():
    spec = importlib.util.spec_from_file_location('fast_v7_runner_test', RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_feeder(monkeypatch):
    rosbag = types.ModuleType('rosbag')
    rosbag.ROSBagException = RuntimeError
    rospy = types.ModuleType('rospy')
    rospy.ROSException = RuntimeError
    rospy.ROSInterruptException = RuntimeError
    rospy.is_shutdown = lambda: False
    srv = types.ModuleType('std_srvs.srv')
    srv.Trigger = object
    std_srvs = types.ModuleType('std_srvs')
    std_srvs.srv = srv
    monkeypatch.setitem(sys.modules, 'rosbag', rosbag)
    monkeypatch.setitem(sys.modules, 'rospy', rospy)
    monkeypatch.setitem(sys.modules, 'std_srvs', std_srvs)
    monkeypatch.setitem(sys.modules, 'std_srvs.srv', srv)
    spec = importlib.util.spec_from_file_location('fast_v7_feeder_test', FEEDER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_hanging_trigger_surrogate_times_out_without_thread_leak(monkeypatch):
    feeder = _load_feeder(monkeypatch)
    before = {thread.ident for thread in threading.enumerate()}

    def hanging_service():
        time.sleep(2.0)

    started = time.monotonic()
    with pytest.raises(feeder.DeadlineExceeded, match='status RPC'):
        feeder._bounded_call(hanging_service, feeder.Deadline(0.05), 'consumer status RPC')
    assert time.monotonic() - started < 0.5
    after = {thread.ident for thread in threading.enumerate()}
    assert after == before


def test_hanging_publish_surrogate_times_out_under_same_transaction_deadline(monkeypatch):
    feeder = _load_feeder(monkeypatch)
    deadline = feeder.Deadline(0.05)

    class HangingPublisher:
        def publish(self, _message):
            time.sleep(2.0)

    with pytest.raises(feeder.DeadlineExceeded, match='publish'):
        feeder._bounded_call(
            lambda: HangingPublisher().publish(object()), deadline, 'publish /points'
        )


def test_failure_diagnostic_is_immutable(monkeypatch, tmp_path):
    feeder = _load_feeder(monkeypatch)
    feeder.write_failure_diagnostic(tmp_path, feeder.DeadlineExceeded('first'))
    first = (tmp_path / 'feeder_failure.json').read_bytes()
    feeder.write_failure_diagnostic(tmp_path, feeder.DeadlineExceeded('second'))
    assert (tmp_path / 'feeder_failure.json').read_bytes() == first
    assert not list(tmp_path.glob('*.part'))
    assert json.loads(first)['status'] == 'FAIL_CLOSED'


def test_watchdog_captures_inspect_stats_top_then_term_and_kill(monkeypatch, tmp_path):
    runner = _load_runner()
    state = {'stopped': False, 'killed': False}

    class Process:
        pid = 123

        def poll(self):
            return 124 if state['stopped'] or state['killed'] else None

        def wait(self):
            return 124

    def inspect(_name):
        if state['killed']:
            return {'status': 'present', 'running': False, 'exit_code': 137}
        if state['stopped']:
            # Surrogate ignores TERM, forcing the watchdog's kill path.
            return {'status': 'present', 'running': True, 'exit_code': 0}
        return {'status': 'present', 'running': True, 'exit_code': 0}

    calls = []
    monkeypatch.setattr(runner, 'docker_container_snapshot', inspect)
    monkeypatch.setattr(
        runner,
        'docker_stats_snapshot',
        lambda _name: {'status': 'present', 'memory_usage_bytes': 7},
    )
    monkeypatch.setattr(
        runner, 'docker_top_snapshot', lambda _name: {'status': 'present', 'stdout': 'surrogate'}
    )

    def stop(_name, _grace):
        calls.append('stop')
        state['stopped'] = True
        return {'status': 'requested', 'returncode': 0}

    def kill(_name):
        calls.append('kill')
        state['killed'] = True
        return {'status': 'requested', 'returncode': 0}

    monkeypatch.setattr(runner, 'docker_stop', stop)
    monkeypatch.setattr(runner, 'docker_kill', kill)
    monkeypatch.setattr(runner.time, 'sleep', lambda _seconds: None)
    destination = tmp_path / 'host_lifecycle.json'
    cidfile = tmp_path / 'container.cid'
    cidfile.write_text('cid\n', encoding='utf-8')
    status, lifecycle = runner.supervise_container(
        Process(),
        container_name='fast-v7-surrogate',
        cidfile=cidfile,
        lifecycle_path=destination,
        interval_seconds=0.001,
        stats_interval_seconds=0.001,
        max_runtime_seconds=0.001,
        term_grace_seconds=0.01,
    )
    assert status == 124
    assert calls == ['stop', 'kill']
    assert lifecycle['status'] == 'watchdog_fail_closed'
    assert lifecycle['watchdog']['triggered'] is True
    assert lifecycle['watchdog']['diagnostics']['top']['stdout'] == 'surrogate'
    assert lifecycle['watchdog']['kill']['returncode'] == 0
    assert json.loads(destination.read_text())['status'] == 'watchdog_fail_closed'
    assert not list(tmp_path.glob('*.part'))


def test_v7_profile_is_unexecuted_and_binds_new_image_labels():
    value = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))['competitive_slam_profile'][
        'm6a10_fast_livo2_v2c_v7'
    ]
    assert value['status'] == 'preregistered_not_executed'
    assert value['result'] is None
    assert value['execution']['status'] == 'not_built'
    assert value['execution']['image_digest'] is None
    assert value['execution']['image_reuse_forbidden'] is True
    assert value['watchdog']['max_runtime_seconds'] == 900
    assert value['watchdog']['diagnostics_before_stop'] == [
        'docker_inspect',
        'docker_stats',
        'docker_top',
    ]
    assert value['runner']['run_json_written_on_outer_agent_loss'] is True
    assert value['feeder']['rpc_timeout_thread_leak_free'] is True
    assert value['image_labels']['feeder_sha256'] != (
        'bde0631d29dbb18575fe0fd2ce4bc339e738d14e1b47a1ccc8a77aa348f5622d'
    )
    assert (
        value['image_labels']['runner_sha256']
        == hashlib.sha256(RUNNER_PATH.read_bytes()).hexdigest()
    )


def test_detached_watchdog_writes_orphan_run_receipt_after_container_exit(monkeypatch, tmp_path):
    runner = _load_runner()
    monkeypatch.setattr(
        runner,
        'docker_container_snapshot',
        lambda _name: {'status': 'present', 'running': False, 'exit_code': 137},
    )
    monkeypatch.setattr(
        runner,
        'docker_stats_snapshot',
        lambda _name: {'status': 'present', 'memory_usage_bytes': 1},
    )
    monkeypatch.setattr(runner.time, 'sleep', lambda _seconds: None)
    lifecycle = tmp_path / 'host_lifecycle.json'
    run_json = tmp_path / 'run.json'
    code = runner._detached_watchdog_loop(
        container_name='fast-v7-orphan',
        cidfile=tmp_path / 'cid',
        lifecycle_path=lifecycle,
        run_json_path=run_json,
        command_sha256='c' * 64,
        input_path='/input/raw_input.bag',
        input_sha256='b' * 64,
        contract_id='m6a10-v2c-fast-livo2-single-inflight-v2-watchdog-v1',
        max_runtime_seconds=60,
        term_grace_seconds=1,
        interval_seconds=0.01,
        stats_interval_seconds=0.01,
    )
    assert code == 137
    assert json.loads(lifecycle.read_text())['status'] == 'container_exited'
    receipt = json.loads(run_json.read_text())
    assert receipt['status'] == 'FAIL_CLOSED'
    assert receipt['failure'] == 'outer_host_runner_missing_before_run_receipt'
    assert receipt['execution']['scorer_invoked'] is False
    assert not list(tmp_path.glob('*.part'))
