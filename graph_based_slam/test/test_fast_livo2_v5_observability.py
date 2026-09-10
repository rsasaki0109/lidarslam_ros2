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

"""Regression tests for FAST v5 observability without Docker or bag replay."""

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SELECTION = ROOT / (
    'configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml')
FEEDER = ROOT / 'scripts/fast_livo2_m6a10_feeder.py'
RUNNER_PATH = ROOT / 'scripts/run_fast_livo2_benchmark.py'


def _load_runner():
    spec = importlib.util.spec_from_file_location('fast_v5_runner', RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _contract(path):
    return yaml.safe_load(path.read_text())['competitive_slam_profile'][
        'm6a10_fast_livo2_v2c']


def test_v5_is_preregistered_and_keeps_v4_failure_lineage():
    profile = _contract(PROFILE)
    selection = yaml.safe_load(SELECTION.read_text())['m6a10_fast_livo2_v2c']
    retry = profile['retry_v5']
    assert retry == selection['retry_v5']
    assert retry['status'] == 'build_passed_not_executed'
    assert retry['result'] is None
    assert retry['runner_start_attempted'] is False
    assert retry['container_start_attempted'] is False
    assert retry['bag_replay_started'] is False
    assert retry['gt_content_opened'] is False
    assert retry['scorer_invoked'] is False
    assert retry['predecessor_receipt_sha256'] == (
        '861585b554cee8240bd4cae811dbec46c7256b83ba40a30c4e28a91ca690c93f')
    assert retry['observability']['feeder_progress_authoritative'] is False
    assert retry['observability']['progress_checkpoint_after_first_ack'] is True
    assert retry['observability']['stable_container_name'] is True
    assert retry['observability']['docker_stats_interval_seconds'] == 5.0
    assert retry['observability']['docker_stats_diagnostic_only'] is True
    assert retry['observability']['container_auto_remove'] is False
    assert retry['observability']['container_cleanup_after_receipt'] is True
    assert retry['observability'][
        'interrupted_supervision_preserves_container'] is True
    assert retry['image']['status'] == 'build_passed_not_executed'
    assert retry['image']['tag'] == (
        'm6a10-v2c-v5portable-fast-livo2-benchmark:ros1-pinned')
    assert retry['image']['image_id'] == (
        'sha256:98271cfa19056d985b982d1fe3af74ea807ab195137b00ba313f0d94f8387849')
    assert retry['image']['active_forbidden_compile_flags'] is False
    assert retry['build']['receipt_sha256'] == retry['image']['build_receipt_sha256']
    assert retry['preflight']['status'] == 'PASS'
    assert retry['preflight']['receipt_sha256'] == (
        '184cd387a567af610bab376d5ba445b347357470b97c98caea5a26fd1dde98cd')
    quiescence = profile['quiescence_attempt_v5']
    assert quiescence['status'] == 'PASS'
    assert quiescence['runner_start_allowed'] is True
    assert quiescence['receipt_sha256'] == (
        'b2ecc273ec82d8a1bc915458e43ffe262af81f9df2aa815258bdd48f6a326762')
    assert quiescence['retry_count'] == 0
    assert quiescence['cpu_busy_percent'] <= quiescence['max_cpu_busy_percent']
    assert quiescence['load1_per_cpu'] <= quiescence['max_load1_per_cpu']
    assert quiescence['forbidden_processes'] == []


def test_v5_hashes_bind_current_instrumentation():
    profile = _contract(PROFILE)
    retry = profile['retry_v5']
    assert retry['feeder']['sha256'] == hashlib.sha256(
        FEEDER.read_bytes()).hexdigest()
    assert retry['runner']['sha256'] == hashlib.sha256(
        RUNNER_PATH.read_bytes()).hexdigest()
    assert profile['feeder']['sha256'] == retry['feeder']['sha256']
    assert profile['runner']['sha256'] == retry['runner']['sha256']
    build_receipt = json.loads(Path(
        retry['build']['receipt_path']).read_text(encoding='utf-8'))
    assert retry['build']['recipe']['sha256'] == (
        build_receipt['build']['dockerfile_sha256'])
    assert retry['build']['entrypoint']['sha256'] == (
        build_receipt['build']['build_script_sha256'])


def test_v5_output_policy_forbids_map_artifacts_but_allows_mapper_diagnostics():
    profile = _contract(PROFILE)
    selection = yaml.safe_load(SELECTION.read_text())['m6a10_fast_livo2_v2c']
    output = profile['retry_v5']['output_contract']
    assert output == selection['retry_v5']['output_contract']
    assert output['root'].endswith('fast_livo2_v2c_fixed10_v5')
    assert '*map*' not in output['forbidden_globs']
    assert 'pointcloud_map' in output['forbidden_globs']
    assert '*.pcd' in output['forbidden_globs']
    assert '*.bag' in output['forbidden_globs']
    assert set(output['diagnostic_names_allowed']) == {
        'mapper.log', 'mapper_ready.txt'}
    assert output['diagnostic_names_are_not_map_outputs'] is True


def test_feeder_progress_is_atomic_non_authoritative_and_line_buffered():
    text = FEEDER.read_text()
    assert 'atomic_progress_json' in text
    assert 'authoritative_completion' in text
    assert 'M6A10_FAST_PROGRESS_EVERY' in text
    assert 'sys.stdout.reconfigure(line_buffering=True)' in text
    assert 'FAST feeder ACK record=' in text


def test_feeder_progress_is_bounded_not_per_message():
    text = FEEDER.read_text()
    assert 'if records == 0:' in text
    assert 'if records == 1 or records % progress_every == 0:' in text
    assert 'FAST feeder publish record=' not in text
    assert 'M6A10_FAST_DRAIN_PROGRESS_PATH' in text
    assert "phase='publish_waiting_for_callback'" in text


def test_host_supervision_has_reconnectable_identity_and_atomic_lifecycle():
    runner = _load_runner()
    text = RUNNER_PATH.read_text()
    assert "'--name'" in text
    assert "'--cidfile'" in text
    assert 'host_lifecycle.json' in text
    assert 'reconnect_command' in text
    assert runner.docker_container_snapshot('container-that-does-not-exist')[
        'status'] in {'not_found', 'inspect_error'}
    assert hasattr(runner, 'ensure_container_name_available')
    assert hasattr(runner, 'docker_stats_snapshot')
    assert hasattr(runner, 'remove_exited_container')


def test_stale_name_and_docker_inspect_errors_fail_closed(monkeypatch):
    runner = _load_runner()
    monkeypatch.setattr(
        runner, 'docker_container_snapshot', lambda _name: {'status': 'not_found'})
    runner.ensure_container_name_available('fresh-name')
    for status in ('present', 'inspect_error'):
        monkeypatch.setattr(
            runner, 'docker_container_snapshot', lambda _name, value=status: {
                'status': value})
        with pytest.raises(RuntimeError):
            runner.ensure_container_name_available('unsafe-name')


def test_v5_attached_run_keeps_container_for_post_exit_inspection(tmp_path):
    runner = _load_runner()
    args = type('Args', (), {'phase_mode': 'unpaced_ack'})()
    phase = yaml.safe_load(PROFILE.read_text())[
        'competitive_slam_profile']['m6a10_fast_livo2_v2c']
    command = runner.build_m6a10_fast_command(
        args, phase, tmp_path, Path(phase['input']['path']))
    assert '--rm' not in command
    assert '--cidfile' in command


def test_atomic_host_diagnostic_replaces_without_partial_file(tmp_path):
    runner = _load_runner()
    destination = tmp_path / 'lifecycle.json'
    runner.atomic_json_replace(destination, {'status': 'running'})
    runner.atomic_json_replace(destination, {'status': 'exited'})
    assert json.loads(destination.read_text()) == {'status': 'exited'}
    assert not list(tmp_path.glob('*.part'))


def test_supervision_records_exit_and_does_not_signal_process(tmp_path, monkeypatch):
    runner = _load_runner()

    class FakeProcess:
        pid = 321

        def __init__(self):
            self.poll_count = 0
            self.signalled = False

        def poll(self):
            self.poll_count += 1
            return None if self.poll_count == 1 else 0

        def wait(self):
            return 0

    process = FakeProcess()
    monkeypatch.setattr(runner, 'docker_container_snapshot',
                        lambda _name: {'status': 'present', 'running': True})
    monkeypatch.setattr(runner.time, 'sleep', lambda _seconds: None)
    cidfile = tmp_path / 'container.cid'
    cidfile.write_text('abc123\n')
    destination = tmp_path / 'lifecycle.json'
    status, lifecycle = runner.supervise_container(
        process, container_name='m6a10-v5-test', cidfile=cidfile,
        lifecycle_path=destination, interval_seconds=0.01)
    assert status == 0
    assert lifecycle['status'] == 'exited'
    assert lifecycle['cid'] == 'abc123'
    assert process.signalled is False
    assert json.loads(destination.read_text())['returncode'] == 0
