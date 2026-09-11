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

"""Tests for the bounded product map soak harness."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
from types import SimpleNamespace

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_map_soak',
    ROOT / 'scripts/run_map_soak.py',
)
SOAK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SOAK)


def _schema_v4() -> dict:
    return json.loads(
        (ROOT / 'docs/schemas/soak-report-v4.schema.json').read_text()
    )


def _provenance() -> tuple[dict, dict]:
    return (
        {
            'bag_path': '/data/bag',
            'metadata_path': '/data/bag/metadata.yaml',
            'metadata_size_bytes': 42,
            'metadata_sha256': 'a' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [
                {
                    'path': 'bag_0.db3',
                    'size_bytes': 1024,
                    'sha256': 'b' * 64,
                },
            ],
            'identity_algorithm': 'sha256',
        },
        {
            'product_version': '0.9.0',
            'git_commit': 'c' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.9.0'},
            'ros_distro': 'jazzy',
        },
    )


def _args(tmp_path: Path, **overrides) -> argparse.Namespace:
    bag = tmp_path / 'bag'
    bag.mkdir(exist_ok=True)
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    cli = tmp_path / 'lidarslam-map'
    cli.write_text('#!/bin/sh\n', encoding='utf-8')
    cli.chmod(0o755)
    values = {
        'bag': str(bag),
        'output_root': str(tmp_path / 'soak'),
        'soak_profile': 'one-hour',
        'hardware_label': 'ci-amd64-test-host',
        'map_profile': 'mid360_livox_smoke',
        'cli': str(cli),
        'max_peak_rss_mib': 512.0,
        'max_output_gib': 1.0,
        'max_dropped_inputs': 0,
        'max_iteration_secs': 300.0,
        'min_free_space_gib': 0.01,
        'telemetry_interval_secs': 30.0,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_profiles_are_exactly_one_and_eight_hours():
    assert SOAK.PROFILE_DURATIONS == {
        'one-hour': 3600.0,
        'eight-hour': 28800.0,
    }


def test_telemetry_interval_is_positive_and_at_most_sixty_seconds():
    assert SOAK._telemetry_interval('0.25') == 0.25
    assert SOAK._telemetry_interval('60') == 60.0
    with pytest.raises(argparse.ArgumentTypeError, match='exceed'):
        SOAK._telemetry_interval('60.01')


def test_parse_gnu_time_reports_wall_rss_and_exit():
    report = SOAK.parse_gnu_time(
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 1:02:03.50\n'
        'Maximum resident set size (kbytes): 262144\n'
        'Exit status: 7\n'
    )

    assert report == {
        'wall_time_sec': 3723.5,
        'peak_rss_mib': 256.0,
        'exit_code': 7,
    }


def test_parse_gnu_time_rejects_incomplete_evidence():
    with pytest.raises(ValueError, match='peak_rss'):
        SOAK.parse_gnu_time(
            'Elapsed (wall clock) time (h:mm:ss or m:ss): 2:00.00\n'
            'Exit status: 0\n'
        )


def test_drop_counters_are_explicit_and_conservative():
    counts = SOAK.count_dropped_inputs(
        'Message Filter dropping message because its queue is full\n'
        'Dropping scan during kidnap recovery\n'
        'unrelated warning\n'
    )

    assert counts == {
        'message_filter': 1,
        'scan_drop': 1,
        'queue_overflow': 1,
    }


def test_machine_fingerprint_is_stable_and_does_not_expose_private_ids():
    first = SOAK.capture_machine_fingerprint()
    second = SOAK.capture_machine_fingerprint()

    assert first == second
    assert len(first['machine_id']) == 64
    assert first['logical_cpu_count'] > 0
    assert 'private_identifiers' not in first


def test_iteration_provenance_requires_succeeded_product_manifest(tmp_path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    input_identity, software_identity = _provenance()
    manifest = {
        'status': 'succeeded',
        'input': input_identity,
        'software': software_identity,
    }
    (run_dir / SOAK.RUN_MANIFEST_NAME).write_text(
        json.dumps(manifest),
        encoding='utf-8',
    )

    assert SOAK._load_iteration_provenance(run_dir) == (
        input_identity,
        software_identity,
    )

    manifest['status'] = 'failed'
    (run_dir / SOAK.RUN_MANIFEST_NAME).write_text(
        json.dumps(manifest),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='not succeeded'):
        SOAK._load_iteration_provenance(run_dir)


def test_successful_soak_writes_schema_valid_threshold_evidence(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 3600.0, 3600.0])

    def fake_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        assert telemetry_interval_sec == 30.0
        assert max_iteration_sec == 300.0
        on_sample(0.0)
        run_dir = Path(command[command.index('--output-dir') + 1])
        run_dir.mkdir(parents=True)
        (run_dir / 'map.pcd').write_bytes(b'x' * 1024)
        on_sample(15.0)
        checkpoint = json.loads(
            (Path(args.output_root) / SOAK.REPORT_NAME).read_text()
        )
        assert checkpoint['status'] == 'running'
        assert checkpoint['active_iteration_id'] == 'iteration-0001'
        assert len(checkpoint['telemetry_samples']) == 2
        jsonschema.validate(checkpoint, _schema_v4())
        log_path.write_text('', encoding='utf-8')
        time_path.write_text('fake GNU time evidence\n', encoding='utf-8')
        return (
            0,
            {'wall_time_sec': 30.0, 'peak_rss_mib': 128.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
        load_iteration_provenance=lambda run_dir: _provenance(),
    )

    assert exit_code == 0
    assert report['status'] == 'passed'
    assert report['metrics']['iterations_completed'] == 1
    assert report['metrics']['wall_time_sec'] == 3600.0
    assert len(report['telemetry_samples']) == 2
    assert report['telemetry_samples'][1]['output_size_bytes'] == 1024
    assert report['active_iteration_id'] is None
    assert all(report['checks'].values())
    assert report['input_identity']['metadata_sha256'] == 'a' * 64
    assert report['software_identity']['git_commit'] == 'c' * 40
    assert report['hardware']['machine_id']
    assert report['harness']['runner_sha256']
    saved = json.loads((Path(args.output_root) / SOAK.REPORT_NAME).read_text())
    schema = _schema_v4()
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(saved, schema)


def test_failed_iteration_stops_and_preserves_failed_report(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 12.0])

    def fake_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(0.0)
        log_path.write_text('Dropping scan: queue full\n', encoding='utf-8')
        time_path.write_text('fake GNU time evidence\n', encoding='utf-8')
        return (
            9,
            {'wall_time_sec': 12.0, 'peak_rss_mib': 64.0, 'exit_code': 9},
            {'message_filter': 0, 'scan_drop': 1, 'queue_overflow': 1},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['metrics']['iterations_failed'] == 1
    assert report['metrics']['dropped_inputs_total'] == 2
    assert report['checks']['target_duration_reached'] is False
    assert report['checks']['all_iterations_succeeded'] is False
    assert (Path(args.output_root) / SOAK.REPORT_NAME).is_file()


def test_successful_iteration_without_product_provenance_fails_soak(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 12.0])

    def fake_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(0.0)
        return (
            0,
            {'wall_time_sec': 12.0, 'peak_rss_mib': 64.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['checks']['provenance_recorded'] is False
    assert 'lacks run_manifest.json' in report['last_error']
    jsonschema.validate(report, _schema_v4())


def test_changed_input_identity_between_iterations_fails_soak(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 10.0, 10.0, 20.0])
    calls = []

    def fake_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        calls.append(command)
        on_sample(0.0)
        return (
            0,
            {'wall_time_sec': 10.0, 'peak_rss_mib': 64.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    provenance_calls = 0

    def changing_provenance(run_dir):
        nonlocal provenance_calls
        provenance_calls += 1
        input_identity, software_identity = _provenance()
        if provenance_calls == 2:
            input_identity = {
                **input_identity,
                'metadata_sha256': 'd' * 64,
            }
        return input_identity, software_identity

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=fake_iteration,
        load_iteration_provenance=changing_provenance,
    )

    assert exit_code == 1
    assert len(calls) == 2
    assert report['status'] == 'failed'
    assert report['metrics']['iterations_completed'] == 1
    assert report['metrics']['iterations_failed'] == 1
    assert 'input identity differs' in report['last_error']


def test_telemetry_failure_becomes_terminal_evidence(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 3.0])

    def broken_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        raise ValueError('GNU time report lacks fields')

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=broken_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['completed_at'] is not None
    assert 'lacks fields' in report['last_error']
    saved = json.loads((Path(args.output_root) / SOAK.REPORT_NAME).read_text())
    assert saved['status'] == 'failed'
    assert saved['last_error'] == report['last_error']


def test_resource_budget_failure_stops_before_another_iteration(tmp_path):
    args = _args(tmp_path, max_peak_rss_mib=100.0)
    ticks = iter([0.0, 0.0, 10.0])
    calls = []

    def over_budget_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(0.0)
        calls.append(command)
        return (
            0,
            {'wall_time_sec': 10.0, 'peak_rss_mib': 101.0, 'exit_code': 0},
            {'message_filter': 0, 'scan_drop': 0, 'queue_overflow': 0},
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=over_budget_iteration,
        load_iteration_provenance=lambda run_dir: _provenance(),
    )

    assert exit_code == 1
    assert len(calls) == 1
    assert report['checks']['peak_rss_within_budget'] is False
    assert report['last_error'].endswith('failed peak_rss_within_budget')


def test_periodic_low_space_sample_aborts_and_preserves_evidence(
    tmp_path,
    monkeypatch,
):
    args = _args(tmp_path, min_free_space_gib=1.0)
    ticks = iter([0.0, 0.0, 4.0])
    monkeypatch.setattr(
        SOAK.shutil,
        'disk_usage',
        lambda path: SimpleNamespace(free=512 * 1024**2),
    )

    def sample_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(4.0)
        raise AssertionError('low-space callback must abort the iteration')

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=sample_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['metrics']['iterations_failed'] == 1
    assert report['metrics']['minimum_free_space_bytes'] == 512 * 1024**2
    assert len(report['telemetry_samples']) == 1
    assert report['checks']['free_space_within_budget'] is False
    assert 'fell below minimum_free_space_bytes' in report['last_error']
    saved = json.loads((Path(args.output_root) / SOAK.REPORT_NAME).read_text())
    assert saved['telemetry_samples'] == report['telemetry_samples']


def test_periodic_output_growth_aborts_before_iteration_finishes(tmp_path):
    args = _args(tmp_path, max_output_gib=1.0 / 1024**3)
    ticks = iter([0.0, 0.0, 2.0])

    def growing_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        run_dir = Path(command[command.index('--output-dir') + 1])
        run_dir.mkdir(parents=True)
        (run_dir / 'growing.map').write_bytes(b'xx')
        on_sample(2.0)
        raise AssertionError('output-budget callback must abort the iteration')

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=growing_iteration,
    )

    assert exit_code == 1
    assert report['metrics']['output_size_bytes'] == 2
    assert report['checks']['output_size_within_budget'] is False
    assert 'failed output_size_within_budget' in report['last_error']


def test_iteration_callback_failure_terminates_process_group(tmp_path):
    log_path = tmp_path / 'iteration.log'
    time_path = tmp_path / 'iteration.time'
    started = time.monotonic()

    with pytest.raises(RuntimeError, match='stop now'):
        SOAK._run_iteration(
            [sys.executable, '-c', 'import time; time.sleep(30)'],
            log_path,
            time_path,
            telemetry_interval_sec=0.01,
            max_iteration_sec=1.0,
            on_sample=lambda elapsed: (_ for _ in ()).throw(
                RuntimeError('stop now')
            ),
        )

    assert time.monotonic() - started < 2.0


def test_process_group_cleanup_kills_survivor_after_leader_exits(
    monkeypatch,
):
    child_code = (
        'import signal, time; '
        'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
        'time.sleep(30)'
    )
    leader_code = (
        'import subprocess, sys, time; '
        f'child = subprocess.Popen([sys.executable, "-c", {child_code!r}]); '
        'time.sleep(0.1); '
        'print(child.pid, flush=True); '
        'time.sleep(30)'
    )
    process = SOAK.subprocess.Popen(
        [sys.executable, '-c', leader_code],
        start_new_session=True,
        stdout=SOAK.subprocess.PIPE,
        text=True,
    )
    child_pid = int(process.stdout.readline())
    monkeypatch.setattr(SOAK, 'PROCESS_SHUTDOWN_GRACE_SECS', 0.05)

    try:
        SOAK._terminate_process_group(process)
        child_stat = Path(f'/proc/{child_pid}/stat')
        deadline = time.monotonic() + 1.0
        while child_stat.exists() and time.monotonic() < deadline:
            if child_stat.read_text().split()[2] == 'Z':
                break
            time.sleep(0.01)
        assert (
            not child_stat.exists()
            or child_stat.read_text().split()[2] == 'Z'
        )
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def test_real_iteration_emits_periodic_and_final_samples(tmp_path):
    log_path = tmp_path / 'iteration.log'
    time_path = tmp_path / 'iteration.time'
    samples = []

    return_code, resources, dropped = SOAK._run_iteration(
        [sys.executable, '-c', 'import time; time.sleep(0.08)'],
        log_path,
        time_path,
        telemetry_interval_sec=0.02,
        max_iteration_sec=1.0,
        on_sample=samples.append,
    )

    assert return_code == 0
    assert resources['exit_code'] == 0
    assert resources['wall_time_sec'] >= 0.08
    assert dropped == {
        'message_filter': 0,
        'scan_drop': 0,
        'queue_overflow': 0,
    }
    assert samples[0] == 0.0
    assert len(samples) >= 3
    assert samples == sorted(samples)


def test_real_iteration_timeout_terminates_process_group(tmp_path):
    log_path = tmp_path / 'iteration.log'
    time_path = tmp_path / 'iteration.time'
    samples = []
    started = time.monotonic()

    with pytest.raises(TimeoutError, match='max_iteration_sec'):
        SOAK._run_iteration(
            [sys.executable, '-c', 'import time; time.sleep(30)'],
            log_path,
            time_path,
            telemetry_interval_sec=30.0,
            max_iteration_sec=0.05,
            on_sample=samples.append,
        )

    assert time.monotonic() - started < 2.0
    assert samples[0] == 0.0
    assert samples[-1] >= 0.05


def test_iteration_timeout_is_terminal_failed_evidence(tmp_path):
    args = _args(tmp_path, max_iteration_secs=10.0)
    ticks = iter([0.0, 0.0, 10.1])

    def timed_out_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        assert max_iteration_sec == 10.0
        on_sample(10.1)
        raise TimeoutError(
            'iteration exceeded max_iteration_sec (10)'
        )

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=timed_out_iteration,
    )

    assert exit_code == 1
    assert report['status'] == 'failed'
    assert report['metrics']['iterations_failed'] == 1
    assert report['metrics']['max_observed_iteration_sec'] == 10.1
    assert report['checks']['iteration_duration_within_budget'] is False
    assert report['last_error'].endswith(
        'iteration exceeded max_iteration_sec (10)'
    )
    jsonschema.validate(report, _schema_v4())


def test_real_iteration_forwards_sigterm_and_restores_handler(tmp_path):
    log_path = tmp_path / 'iteration.log'
    time_path = tmp_path / 'iteration.time'
    previous_handler = signal.getsignal(signal.SIGTERM)
    timer = threading.Timer(
        0.1,
        lambda: os.kill(os.getpid(), signal.SIGTERM),
    )
    started = time.monotonic()
    timer.start()
    try:
        with pytest.raises(SOAK.TerminationSignal) as raised:
            SOAK._run_iteration(
                [sys.executable, '-c', 'import time; time.sleep(30)'],
                log_path,
                time_path,
                telemetry_interval_sec=0.02,
                max_iteration_sec=1.0,
                on_sample=lambda elapsed: None,
            )
    finally:
        timer.cancel()

    assert raised.value.signum == signal.SIGTERM
    assert signal.getsignal(signal.SIGTERM) == previous_handler
    assert time.monotonic() - started < 2.0


def test_keyboard_interrupt_is_terminal_and_returns_130(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 2.0])

    def interrupted_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(1.0)
        raise KeyboardInterrupt

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=interrupted_iteration,
    )

    assert exit_code == 130
    assert report['status'] == 'interrupted'
    assert report['metrics']['iterations_failed'] == 1
    assert report['active_iteration_id'] is None
    assert report['last_error'].endswith('interrupted by SIGINT')


def test_sigterm_is_terminal_and_returns_143(tmp_path):
    args = _args(tmp_path)
    ticks = iter([0.0, 0.0, 2.0])

    def terminated_iteration(
        command,
        log_path,
        time_path,
        *,
        telemetry_interval_sec,
        max_iteration_sec,
        on_sample,
    ):
        on_sample(1.0)
        raise SOAK.TerminationSignal(signal.SIGTERM)

    exit_code, report = SOAK.run_soak(
        args,
        clock=lambda: next(ticks),
        run_iteration=terminated_iteration,
    )

    assert exit_code == 143
    assert report['status'] == 'interrupted'
    assert report['metrics']['iterations_failed'] == 1
    assert report['active_iteration_id'] is None
    assert report['last_error'].endswith('interrupted by SIGTERM')
    jsonschema.validate(report, _schema_v4())


def test_existing_output_is_never_overwritten(tmp_path):
    args = _args(tmp_path)
    output_root = Path(args.output_root)
    output_root.mkdir()
    marker = output_root / 'owned'
    marker.write_text('keep', encoding='utf-8')

    with pytest.raises(ValueError, match='already exists'):
        SOAK.run_soak(args)

    assert marker.read_text(encoding='utf-8') == 'keep'
