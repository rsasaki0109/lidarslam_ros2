#!/usr/bin/env python3
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

import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_m6a10_fixed10_v7.py'
SPEC = importlib.util.spec_from_file_location('m6a10_fixed10_v7', SCRIPT)
LAUNCHER = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = LAUNCHER
SPEC.loader.exec_module(LAUNCHER)


def _config(tmp_path: Path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    return LAUNCHER.LaunchConfig(
        root=tmp_path / 'attempt', repo_root=repo, input_root=input_root,
        expected_input_tree_sha256=None)


def _receipt(path: Path, *, passed: bool) -> None:
    checks = {
        'nproc_positive': True,
        'cpu_busy_within_limit': passed,
        'load1_per_cpu_within_limit': passed,
        'no_forbidden_processes': passed,
    }
    path.write_text(json.dumps({
        'schema_version': 1,
        'contract_version': 'm6a10-quiescence-v1',
        'status': 'PASS' if passed else 'FAIL_CLOSED',
        'runner_start_allowed': passed,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'observation': {
            'sample_seconds': 5.0,
            'nproc': 8,
            'proc_race_skips': 0,
            'forbidden_processes': [],
            'limits': {
                'max_cpu_busy_percent': 5.0,
                'max_load1_per_cpu': 0.5,
            },
            'checks': checks,
            'loadavg': {'load1_per_cpu': 0.1},
            'cpu': {'busy_percent': 1.0},
        },
    }) + '\n', encoding='utf-8')


def _identity(_config):
    return {'stub': True}


def _time_report(config):
    (config.root / 'time-v.txt').write_text(
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 0:01.00\n'
        'User time (seconds): 0.10\n'
        'System time (seconds): 0.02\n'
        'Maximum resident set size (kbytes): 100\n',
        encoding='utf-8')


class _Process:
    pid = 4242

    def __init__(self, returncode=0):
        self.returncode = returncode
        self.signals = []

    def wait(self):
        return self.returncode

    def send_signal(self, signum):
        self.signals.append(signum)

    def terminate(self):
        self.signals.append('terminate')


def _pass_quiescence(events):
    def runner(_config, path):
        events.append('preflight')
        _receipt(path, passed=True)
        return 0
    return runner


def test_pass_starts_fixed_runner_immediately_and_records_pid(tmp_path):
    config = _config(tmp_path)
    events = []
    process = _Process()

    def factory(argv, **_kwargs):
        events.append('popen')
        _time_report(config)
        assert argv[0:4] == ['/usr/bin/time', '-v', '-o', str(config.root / 'time-v.txt')]
        assert argv[4:7] == ['docker', 'run', '--rm']
        assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
        assert 'taskset' not in argv
        assert '--cpuset-cpus' not in argv
        assert any(item.startswith('M6A10_SKIP_MAP_SAVE=1') for item in argv)
        assert any(item.startswith('M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1') for item in argv)
        assert any(LAUNCHER.IMAGE_DIGEST in item for item in argv)
        return process

    assert LAUNCHER.run_v7(
        config, quiescence_runner=_pass_quiescence(events),
        process_factory=factory, identity_validator=_identity,
        completion_validator=lambda _path: {'status': 'PASS'}) == 0
    assert events == ['preflight', 'popen']
    started = json.loads((config.root / 'launch_started.json').read_text())
    assert started['child_pid'] == 4242
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['status'] == 'COMPLETED'
    assert closure['failure_kind'] == 'COMPLETED'
    assert not (config.root / 'closure_receipt.json.part').exists()


def test_preflight_failure_never_starts_runner(tmp_path):
    config = _config(tmp_path)
    called = []

    def factory(*_args, **_kwargs):
        called.append(True)
        raise AssertionError('runner must not start')

    def failed(_config, path):
        _receipt(path, passed=False)
        return 1

    assert LAUNCHER.run_v7(
        config, quiescence_runner=failed, process_factory=factory,
        identity_validator=_identity) == 10
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert called == []
    assert not (config.root / 'launch_attempt.json').exists()


def test_identity_failure_happens_before_quiescence_and_runner(tmp_path):
    config = _config(tmp_path)
    events = []

    def identity(_config):
        events.append('identity')
        raise LAUNCHER.LaunchError('SOURCE_IDENTITY_MISMATCH', 'synthetic source drift')

    def preflight(_config, _path):
        events.append('preflight')
        raise AssertionError('quiescence must not run after identity failure')

    assert LAUNCHER.run_v7(
        config, identity_validator=identity, quiescence_runner=preflight) == 11
    assert events == ['identity']
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'IDENTITY_PREFLIGHT_FAIL_CLOSED'


def test_malformed_pass_receipt_is_not_authorization(tmp_path):
    config = _config(tmp_path)

    def malformed(_config, path):
        _receipt(path, passed=True)
        value = json.loads(path.read_text())
        value['observation']['forbidden_processes'] = [{'class': 'compiler'}]
        path.write_text(json.dumps(value), encoding='utf-8')
        return 0

    assert LAUNCHER.run_v7(
        config, identity_validator=_identity,
        quiescence_runner=malformed,
        process_factory=lambda *_a, **_k: pytest.fail('runner must not start')) == 10
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'


def test_preflight_to_runner_gap_is_fail_closed(tmp_path, monkeypatch):
    config = _config(tmp_path)
    process = _Process()
    stamps = iter((100, 100 + LAUNCHER.MAX_PREFLIGHT_TO_RUN_GAP_NS + 1))
    monkeypatch.setattr(LAUNCHER.time, 'monotonic_ns', lambda: next(stamps))
    assert LAUNCHER.run_v7(
        config, identity_validator=_identity,
        quiescence_runner=_pass_quiescence([]),
        process_factory=lambda *_a, **_k: process,
        completion_validator=lambda _path: pytest.fail(
            'completion must not run after continuity failure')) == 22
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_RUN_GAP_FAIL_CLOSED'
    assert closure['gap_ns'] == LAUNCHER.MAX_PREFLIGHT_TO_RUN_GAP_NS + 1


def test_zero_exit_without_required_artifacts_is_not_completion(tmp_path):
    config = _config(tmp_path)

    def factory(*_args, **_kwargs):
        _time_report(config)
        return _Process()

    assert LAUNCHER.run_v7(
        config, identity_validator=_identity,
        quiescence_runner=_pass_quiescence([]),
        process_factory=factory) == 32
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'COMPLETION_CONTRACT_FAIL_CLOSED'


def test_runner_start_failure_is_distinct(tmp_path):
    config = _config(tmp_path)

    def factory(*_args, **_kwargs):
        raise OSError('synthetic docker spawn failure')

    assert LAUNCHER.run_v7(
        config, quiescence_runner=_pass_quiescence([]),
        process_factory=factory, identity_validator=_identity) == 20
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'RUNNER_START_FAILURE'
    assert json.loads((config.root / 'launch_attempt.json').read_text())[
        'start_attempted'] is True


def test_signal_and_exception_closures_are_distinct(tmp_path):
    signal_base = tmp_path / 'signal'
    signal_base.mkdir()
    signal_config = _config(signal_base)

    class InterruptProcess(_Process):
        calls = 0

        def wait(self):
            self.calls += 1
            if self.calls == 1:
                raise KeyboardInterrupt
            return 130

    assert LAUNCHER.run_v7(
        signal_config, quiescence_runner=_pass_quiescence([]),
        process_factory=lambda *_a, **_k: InterruptProcess(),
        identity_validator=_identity) == 130
    signal_closure = json.loads(
        (signal_config.root / 'closure_receipt.json').read_text())
    assert signal_closure['failure_kind'] == 'LAUNCHER_SIGNAL'

    exception_base = tmp_path / 'exception'
    exception_base.mkdir()
    exception_config = _config(exception_base)

    class BrokenProcess(_Process):
        def wait(self):
            raise RuntimeError('synthetic wait failure')

    assert LAUNCHER.run_v7(
        exception_config, quiescence_runner=_pass_quiescence([]),
        process_factory=lambda *_a, **_k: BrokenProcess(),
        identity_validator=_identity) == 70
    exception_closure = json.loads(
        (exception_config.root / 'closure_receipt.json').read_text())
    assert exception_closure['failure_kind'] == 'LAUNCHER_EXCEPTION'


def test_existing_root_and_rerun_are_rejected_without_overwrite(tmp_path):
    config = _config(tmp_path)
    config.root.mkdir()
    (config.root / 'stale.txt').write_text('stale', encoding='utf-8')
    assert LAUNCHER.run_v7(config, quiescence_runner=_pass_quiescence([])) == 2
    assert not (config.root / 'closure_receipt.json').exists()

    clean_config = _config(tmp_path / 'clean')

    def clean_factory(*_args, **_kwargs):
        _time_report(clean_config)
        return _Process()

    assert LAUNCHER.run_v7(
        clean_config, quiescence_runner=_pass_quiescence([]),
        process_factory=clean_factory, identity_validator=_identity,
        completion_validator=lambda _path: {'status': 'PASS'}) == 0
    before = (clean_config.root / 'closure_receipt.json').read_bytes()
    assert LAUNCHER.run_v7(
        clean_config, quiescence_runner=_pass_quiescence([]),
        process_factory=clean_factory, identity_validator=_identity,
        completion_validator=lambda _path: {'status': 'PASS'}) == 2
    assert (clean_config.root / 'closure_receipt.json').read_bytes() == before


def test_attempt_root_may_not_be_inside_input_or_repository(tmp_path):
    repo = tmp_path / 'repo'
    input_root = tmp_path / 'input'
    repo.mkdir()
    input_root.mkdir()
    for parent in (repo, input_root):
        config = LAUNCHER.LaunchConfig(
            root=parent / 'attempt', repo_root=repo, input_root=input_root,
            expected_input_tree_sha256=None)
        with pytest.raises(LAUNCHER.LaunchError, match='overlap'):
            LAUNCHER.reserve_attempt_root(config)


def test_command_builder_rejects_identity_and_gt_injection(tmp_path):
    config = _config(tmp_path)
    output = config.root / 'out'
    output.mkdir(parents=True)
    with pytest.raises(LAUNCHER.LaunchError, match='image identity'):
        LAUNCHER.build_runner_argv(
            LAUNCHER.LaunchConfig(
                root=config.root, repo_root=config.repo_root,
                input_root=config.input_root, image_tag='evil;docker'), output)


def test_relative_quiescence_script_is_bound_to_repository_root(tmp_path):
    config = _config(tmp_path)
    script = config.repo_root / 'scripts' / 'check.py'
    script.parent.mkdir()
    script.write_text('# fixture\n', encoding='utf-8')
    config = LAUNCHER.LaunchConfig(
        root=config.root, repo_root=config.repo_root, input_root=config.input_root,
        quiescence_script=Path('scripts/check.py'), expected_input_tree_sha256=None)
    command = LAUNCHER._quiescence_command(config, config.root / 'quiescence.json')
    assert command[1] == str(script)
