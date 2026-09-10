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

"""Pure-contract tests for the container PID-namespace RSS sampler."""

import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'sample_container_process_rss',
    ROOT / 'scripts' / 'sample_container_process_rss.py')
SAMPLER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SAMPLER)


STATUS = """Name: fixture
VmRSS:       10 kB
RssAnon:      4 kB
RssFile:      5 kB
RssShmem:     1 kB
"""


def test_proc_status_parser_converts_kib_to_bytes_and_rejects_malformed():
    parsed = SAMPLER.parse_proc_status(STATUS)
    assert parsed == {
        'VmRSS': 10 * 1024, 'RssAnon': 4 * 1024,
        'RssFile': 5 * 1024, 'RssShmem': 1024}
    with pytest.raises(SAMPLER.SamplerError):
        SAMPLER.parse_proc_status(STATUS.replace('10 kB', '10 bytes'))
    with pytest.raises(SAMPLER.SamplerError):
        SAMPLER.parse_proc_status(STATUS.replace('RssShmem:     1 kB\n', ''))


def test_default_sampler_interval_matches_preregistered_contract():
    args = SAMPLER.parse_args(['--output', '/tmp/rss.json'])
    assert args.interval_ms == 250.0
    assert args.scheduler_nice == 10


def test_scan_proc_sums_components_and_excludes_sampler(tmp_path):
    (tmp_path / '101').mkdir()
    (tmp_path / '101' / 'status').write_text(STATUS)
    (tmp_path / '102').mkdir()
    (tmp_path / '102' / 'status').write_text(STATUS.replace('10 kB', '20 kB'))
    current = SAMPLER.os.getpid()
    (tmp_path / str(current)).mkdir()
    (tmp_path / str(current) / 'status').write_text(STATUS.replace('10 kB', '999 kB'))
    sample = SAMPLER.scan_proc(tmp_path)
    assert sample['process_count'] == 2
    assert sample['vmrss_bytes'] == 30 * 1024
    assert sample['rss_anon_bytes'] == 8 * 1024
    assert sample['rss_file_bytes'] == 10 * 1024
    assert sample['rss_shmem_bytes'] == 2 * 1024


def test_scan_proc_counts_pid_races_and_malformed_status(tmp_path):
    (tmp_path / '201').mkdir()
    (tmp_path / '201' / 'status').write_text('not a proc status\n')
    (tmp_path / '202').mkdir()
    sample = SAMPLER.scan_proc(tmp_path)
    assert sample['process_count'] == 0
    assert sample['errors'] == 1
    assert sample['pid_race_skips'] == 1


def test_pid_start_time_parser_is_available_for_reuse_detection(tmp_path):
    directory = tmp_path / '401'
    directory.mkdir()
    fields = ' '.join(['0'] * 18 + ['77'])
    (directory / 'stat').write_text(f'401 (fixture) S {fields}\n')
    assert SAMPLER._pid_start_time(directory) == 77


def test_scan_proc_reuses_one_cross_sample_stat_read_for_pid_reuse_guard(tmp_path):
    directory = tmp_path / '401'
    directory.mkdir()
    (directory / 'status').write_text(STATUS)
    fields = ' '.join(['0'] * 18 + ['77'])
    (directory / 'stat').write_text(f'401 (fixture) S {fields}\n')
    known = {}
    first = SAMPLER.scan_proc(tmp_path, pid_start_times=known)
    assert first['process_count'] == 1
    assert first['pid_reuse_skips'] == 0
    fields = ' '.join(['0'] * 18 + ['88'])
    (directory / 'stat').write_text(f'401 (fixture) S {fields}\n')
    second = SAMPLER.scan_proc(tmp_path, pid_start_times=known)
    assert second['process_count'] == 0
    assert second['pid_race_skips'] == 1
    assert second['pid_reuse_skips'] == 1


def _samples():
    return [{
        'timestamp_monotonic_ns': index * 200_000_000,
        'process_count': 2,
        'vmrss_bytes': 100 + index,
        'rss_anon_bytes': 50,
        'rss_file_bytes': 40,
        'rss_shmem_bytes': 10,
    } for index in range(3)]


def test_summary_records_peak_and_rejects_interval_or_sample_thresholds():
    summary = SAMPLER.summarize_samples(
        _samples(), requested_interval_seconds=0.2, sampler_pid=99,
        sample_errors=0, pid_race_skips=0, min_samples=2, max_errors=0,
        max_race_skips=0, max_jitter_percent=1.0, missed_intervals=0)
    assert summary['status'] == 'pass'
    assert summary['primary_metric'] == \
        'aggregate_process_tree_peak_rss_bytes'
    assert summary['aggregate_process_tree_peak_rss_bytes'] == 102
    assert summary['peak']['vmrss_bytes'] == 102
    assert summary['peak']['timestamp_monotonic_ns'] == 400_000_000
    assert summary['scheduler_nice'] == 0
    invalid = SAMPLER.summarize_samples(
        _samples()[:1], requested_interval_seconds=0.2, sampler_pid=99,
        sample_errors=0, pid_race_skips=0, min_samples=2, max_errors=0,
        max_race_skips=0, max_jitter_percent=1.0, missed_intervals=0)
    assert invalid['status'] == 'invalid'


def test_atomic_report_is_final_only_and_validation_is_fail_closed(tmp_path):
    summary = SAMPLER.summarize_samples(
        _samples(), requested_interval_seconds=0.2, sampler_pid=99,
        sample_errors=0, pid_race_skips=0, min_samples=2, max_errors=0,
        max_race_skips=0, max_jitter_percent=1.0, missed_intervals=0)
    output = tmp_path / 'container_process_rss.json'
    SAMPLER.write_atomic(output, summary)
    assert output.is_file()
    assert not output.with_name(output.name + '.part').exists()
    document = json.loads(output.read_text())
    valid, reason = SAMPLER.validate_summary(document)
    assert valid is True and reason == ''
    with pytest.raises(SAMPLER.SamplerError):
        SAMPLER.write_atomic(output, summary)
    document['sample_count'] = 1
    valid, reason = SAMPLER.validate_summary(document)
    assert valid is False and reason == 'too_few_samples'
    document = json.loads(output.read_text())
    document['primary_metric'] = 'process_tree_peak_rss_bytes'
    valid, reason = SAMPLER.validate_summary(document)
    assert valid is False and reason == 'primary_metric_contract_invalid'


def test_summary_rejects_invalid_scheduler_priority():
    with pytest.raises(SAMPLER.SamplerError, match='scheduler nice'):
        SAMPLER.summarize_samples(
            _samples(), requested_interval_seconds=0.2, sampler_pid=99,
            sample_errors=0, pid_race_skips=0, min_samples=2, max_errors=0,
            max_race_skips=0, max_jitter_percent=1.0, missed_intervals=0,
            scheduler_nice=20)


def test_sigterm_publishes_atomic_report_and_excludes_sampler(tmp_path):
    proc = tmp_path / 'proc'
    proc.mkdir()
    (proc / '301').mkdir()
    (proc / '301' / 'status').write_text(STATUS)
    output = tmp_path / 'container_process_rss.json'
    command = [
        sys.executable, str(ROOT / 'scripts/sample_container_process_rss.py'),
        '--output', str(output), '--proc-root', str(proc), '--interval-ms', '20',
        '--min-samples', '2', '--max-errors', '0', '--max-race-skips', '0',
        '--max-jitter-percent', '100',
    ]
    child = subprocess.Popen(command)
    time.sleep(0.10)
    child.send_signal(signal.SIGTERM)
    assert child.wait(timeout=3) == 0
    assert output.is_file()
    assert not output.with_name(output.name + '.part').exists()
    document = json.loads(output.read_text())
    assert document['atomic'] is True
    assert document['sampler_excluded'] is True
    assert document['peak']['process_count'] == 1
    assert document['scheduler_nice'] == 10
    assert document['scheduler_nice_effective'] >= document['scheduler_nice']
