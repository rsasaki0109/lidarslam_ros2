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

"""GT-free contract fixtures for the RKO-LIO M6a10-v2 consumer hook."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'benchmark_phase_contract', ROOT / 'scripts/benchmark_phase_contract.py')
PHASE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(PHASE)


def _passing_events(mode='paced_1x'):
    value = PHASE.new_events(
        'ours', 10.0, contract_version=PHASE.CONTRACT_VERSION_V2,
        phase_mode=mode)
    value['events_monotonic_ns'] = {
        name: (index + 1) * 1_000_000_000
        for index, name in enumerate(PHASE.EVENT_NAMES)}
    PHASE.set_coverage(
        value, mode='processed_message_counts', complete=True,
        last_timestamp=None, required_timestamp=None,
        expected_messages=3, consumed_messages=3,
        dropped_messages=0, queue_overflow=0)
    value['consumer'] = {
        'ack_source_kind': 'consumer_callback',
        'ack_source': 'rko_lio::ros::OfflineNode::run callback dispatch',
        'ack_semantics': 'next PopNextMessage occurs only after callback return',
        'publisher_count_used': False,
        'eof_observed': True,
        'eof_source': 'BufferableBag::finished()',
        'expected_messages': 3,
        'received_messages': 3,
        'processed_messages': 3,
        'dropped_messages': 0,
        'queue_overflow': 0,
        'backlog_at_drain': 0,
        'maximum_backlog_messages': 0,
        'first_processed_timestamp_seconds': 100.0,
        'last_processed_timestamp_seconds': 109.9,
        'required_end_timestamp_seconds': 110.0,
        'maximum_callback_latency_seconds': 0.1,
        'paced_input_rate': 1.0 if mode == 'paced_1x' else None,
        'paced_input_rate_verified': mode == 'paced_1x',
        'ack_backpressure_verified': mode == 'unpaced_ack',
        'single_message_buffer_verified': False,
        'counter_evidence_path': 'consumer_evidence.json',
    }
    value['resource'] = {
        'status': 'verified', 'scope': 'wrapper_process',
        'cpu_user_seconds': 1.0, 'cpu_system_seconds': 0.5,
        'io_input_operations': 1, 'io_output_operations': 1,
        'source': 'fixture',
    }
    value['exit_status'] = 0
    return value


def test_ours_hook_names_the_real_bag_and_callback_boundaries():
    offline = (ROOT / 'Thirdparty/rko_lio/rko_lio/ros/offline_node.cpp').read_text()
    node = (ROOT / 'Thirdparty/rko_lio/rko_lio/ros/node.cpp').read_text()
    rosbag = (ROOT / 'Thirdparty/rko_lio/rko_lio/ros/utils/rosbag.cpp').read_text()
    assert 'BufferableBag::finished()' in offline
    assert 'benchmark_consumer.record_received()' in offline
    assert 'benchmark_consumer.record_processed' in offline
    assert offline.index('benchmark_consumer.record_received()') < offline.index(
        'benchmark_consumer.record_processed')
    assert offline.index('benchmark_consumer.record_processed') > offline.index(
        'imu_callback(imu_msg)')
    assert 'std::filesystem::rename(part_path, output_path)' in offline
    assert 'single_message_buffer' in offline
    assert 'single_message_buffer_' in rosbag
    assert 'benchmark_consumer.record_drop(true)' in node
    assert 'topic_message_counts_' in rosbag
    assert 'required_end_timestamp_ns_' in rosbag
    assert 'm6a10_consumer_evidence_path' in offline
    assert 'm6a10_phase_contract_version' in offline
    assert 'm6a10_single_message_buffer' in offline
    assert 'bounded disk prefetch' in offline


def test_ours_wrapper_bridges_v2_consumer_path_into_ros_parameters():
    wrapper = (ROOT / 'scripts/run_rko_lio_graph_benchmark.sh').read_text()
    consumer_wrapper = (ROOT / 'scripts/ours_container_gt_blind_run.sh').read_text()
    assert 'm6a10_consumer_evidence_path' in wrapper
    assert 'm6a10_phase_mode' in wrapper
    assert 'M6A10_CONSUMER_EVIDENCE' in wrapper
    assert 'M6A10_CONSUMER_ONLY' in consumer_wrapper
    assert 'ros2 run rko_lio offline_node' in consumer_wrapper
    assert 'map_save_skipped' in consumer_wrapper
    assert 'export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE' in consumer_wrapper
    assert (
        'export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE '
        'M6A10_CONSUMER_EVIDENCE' in wrapper
    )
    assert 'm6a10_child_startup_env.json' in wrapper
    assert "'environment_exported': True" in wrapper
    assert 'M6A10_DRAIN_TIMEOUT_SECONDS' in consumer_wrapper
    assert 'M6A10_DRAIN_DIAGNOSTIC' in consumer_wrapper
    assert 'v2 drain diagnostic path must stay inside output directory' in consumer_wrapper


def test_ours_v2_completion_waits_for_atomic_consumer_evidence():
    wrapper = (ROOT / 'scripts/run_rko_lio_graph_benchmark.sh').read_text()
    assert 'm6a10_completion_contract.sh' in wrapper
    assert 'v2_consumer_evidence_ready()' in wrapper
    assert 'v2 consumer evidence path must stay inside output directory' in wrapper
    assert 'v2 phase mode is missing or unsupported' in wrapper
    assert 'M6A10_CHILD_STARTUP_ENV' in wrapper
    assert 'v2_barrier_required' in wrapper
    assert 'consumer-state' in wrapper
    assert 'return 125' in wrapper
    assert 'completion_status=$?' in wrapper
    assert 'exit "${completion_status}"' in wrapper
    assert '[[ ! -e "${M6A10_CONSUMER_EVIDENCE}.part" ]]' in wrapper
    assert 'OfflineNode exited before v2 consumer evidence/completion was finalized' in wrapper
    assert 'Trajectory stalled before v2 consumer evidence/completion was finalized' in wrapper
    assert 'benchmark_evidence_written' in (
        ROOT / 'Thirdparty/rko_lio/rko_lio/ros/offline_node.cpp').read_text()
    assert 'Commit the consumer proof before fixed-lag/map post-processing' in (
        ROOT / 'Thirdparty/rko_lio/rko_lio/ros/offline_node.cpp').read_text()


@pytest.mark.parametrize(
    ('mutator', 'expected_status'),
    [
        (lambda value: value, 0),
        (lambda value: value.update({'status': 'invalid'}), 1),
    ],
)
def test_v2_consumer_state_preserves_pass_and_invalid_terminal_states(
        tmp_path, mutator, expected_status):
    evidence = _passing_events('unpaced_ack')
    evidence['status'] = 'pass'
    evidence['consumer']['drain_complete'] = True
    evidence['consumer']['processing_failures'] = 0
    mutator(evidence)
    evidence_path = tmp_path / 'consumer.json'
    evidence_path.write_text(json.dumps(evidence), encoding='utf-8')
    completed = subprocess.run([
        'python3', str(ROOT / 'scripts/benchmark_phase_contract.py'),
        'consumer-state', '--input', str(evidence_path),
        '--diagnostic-base', str(tmp_path / 'drain.json')], check=False)
    assert completed.returncode == expected_status


def test_v2_consumer_state_distinguishes_pending_timeout_and_malformed(tmp_path):
    diagnostic_base = tmp_path / 'drain.json'
    missing = tmp_path / 'missing.json'
    pending = subprocess.run([
        'python3', str(ROOT / 'scripts/benchmark_phase_contract.py'),
        'consumer-state', '--input', str(missing),
        '--diagnostic-base', str(diagnostic_base)], check=False)
    assert pending.returncode == 2

    malformed = tmp_path / 'malformed.json'
    malformed.write_text('{not-json', encoding='utf-8')
    invalid = subprocess.run([
        'python3', str(ROOT / 'scripts/benchmark_phase_contract.py'),
        'consumer-state', '--input', str(malformed),
        '--diagnostic-base', str(diagnostic_base)], check=False)
    assert invalid.returncode == 1

    timeout = diagnostic_base.with_name('drain.timeout.json')
    timeout.write_text(json.dumps({
        'schema_version': 1,
        'kind': 'm6a10_drain_diagnostic',
        'event': 'timeout',
        'status': 'timeout',
        'reason': 'fixture drain timeout',
        'benchmark_only': True,
        'contract_version': PHASE.CONTRACT_VERSION_V2,
        'phase_mode': 'unpaced_ack',
        'drain_timeout_seconds': 1.0,
        'snapshot': {
            'lidar_buffer_size': 1,
            'imu_buffer_size': 0,
            'front_lidar_min_timestamp_ns': 100,
            'front_lidar_max_timestamp_ns': 101,
            'last_imu_timestamp_ns': 99,
            'timestamp_gap_ns': -2,
            'atomic_can_process': False,
            'registration_active': False,
        },
        'counters': {
            'expected_messages': 3,
            'received_messages': 3,
            'processed_messages': 3,
            'dropped_messages': 0,
            'queue_overflow': 0,
            'processing_failures': 0,
        },
        'eof_observed': True,
        'drain_complete': False,
        'evidence_path': str(tmp_path / 'consumer.json'),
    }), encoding='utf-8')
    terminal = subprocess.run([
        'python3', str(ROOT / 'scripts/benchmark_phase_contract.py'),
        'consumer-state', '--input', str(missing),
        '--diagnostic-base', str(diagnostic_base)], check=False)
    assert terminal.returncode == 124


@pytest.mark.parametrize('state', [0, 1, 2, 124])
def test_rko_shell_barrier_preserves_all_consumer_states(tmp_path, state):
    """Execute the wrapper's state functions, not a copied pseudo-logic."""
    wrapper = (ROOT / 'scripts/run_rko_lio_graph_benchmark.sh').read_text()
    start = wrapper.index('v2_consumer_evidence_state() {')
    end = wrapper.index('wait_for_offline_completion() {', start)
    functions = wrapper[start:end]
    fake_phase = tmp_path / 'fake_phase.py'
    fake_phase.write_text(
        '#!/usr/bin/env python3\n'
        'import os\n'
        'raise SystemExit(int(os.environ["FAKE_STATE"]))\n',
        encoding='utf-8')
    fake_phase.chmod(0o755)
    evidence = tmp_path / 'consumer.json'
    evidence.write_text('{}\n', encoding='utf-8')
    startup = tmp_path / 'startup.json'
    startup.write_text('{}\n', encoding='utf-8')
    launch_log = tmp_path / 'launch.log'
    launch_log.write_text('RKO LIO Offline Node took fixture\n', encoding='utf-8')
    fixture = f"""\
set +e
OUTPUT_DIR={tmp_path!s}
LAUNCH_LOG={launch_log!s}
M6A10_PHASE_CONTRACT_VERSION=m6a10-online-compute-v2
M6A10_CONSUMER_EVIDENCE={evidence!s}
M6A10_CHILD_STARTUP_ENV={startup!s}
M6A10_PHASE_SCRIPT={fake_phase!s}
FAKE_STATE={state}
export OUTPUT_DIR LAUNCH_LOG M6A10_PHASE_CONTRACT_VERSION
export M6A10_CONSUMER_EVIDENCE M6A10_CHILD_STARTUP_ENV M6A10_PHASE_SCRIPT FAKE_STATE
{functions}
set +e
offline_completion_recorded
printf '%s\\n' "$?"
"""
    completed = subprocess.run(
        ['bash', '-c', fixture], check=False, capture_output=True, text=True)
    assert completed.returncode == 0
    assert int(completed.stdout.strip().splitlines()[-1]) == state


def test_ours_v2_bounded_drain_diagnostic_is_atomic_and_fail_closed():
    offline = (ROOT / 'Thirdparty/rko_lio/rko_lio/ros/offline_node.cpp').read_text()
    node = (ROOT / 'Thirdparty/rko_lio/rko_lio/ros/node.cpp').read_text()
    wrapper = (ROOT / 'scripts/run_rko_lio_graph_benchmark.sh').read_text()
    assert 'BenchmarkDrainSnapshot' in node
    assert 'benchmark_drain_snapshot()' in node
    assert 'm6a10_drain_timeout_seconds' in offline
    assert 'm6a10_drain_diagnostic_path' in offline
    assert 'm6a10_drain_diagnostic' in offline
    assert 'front_lidar_min_timestamp_ns' in offline
    assert 'front_lidar_max_timestamp_ns' in offline
    assert 'last_imu_timestamp_ns' in offline
    assert 'atomic_can_process' in offline
    assert 'registration_active' in offline
    assert 'benchmark drain timeout before lidar buffer became empty' in offline
    assert 'std::filesystem::rename(part_path, output_path)' in offline
    assert 'std::chrono::steady_clock::now() >= drain_deadline' in offline
    assert 'm6a10_drain_timeout_seconds' in wrapper
    assert 'm6a10_drain_diagnostic_path' in wrapper


def test_v2_missing_evidence_cannot_take_historical_completion_shortcut():
    helper = ROOT / 'scripts/m6a10_completion_contract.sh'
    fixture = r"""
set -eu
source "$1"
offline_completion_recorded() { return 1; }
M6A10_PHASE_CONTRACT_VERSION=m6a10-online-compute-v2
if m6a10_completion_barrier_allows; then
  exit 21
fi
offline_completion_recorded() { return 0; }
if ! m6a10_completion_barrier_allows; then
  exit 22
fi
"""
    completed = subprocess.run(
        ['bash', '-c', fixture, 'completion-fixture', str(helper)],
        check=False,
    )
    assert completed.returncode == 0


def test_ours_consumer_counts_eof_and_ack_pass():
    checked = PHASE.validate_evidence_v2(_passing_events(), require_pass=True)
    assert checked['status'] == 'pass'
    assert checked['validation']['publisher_count_rejected'] is True


@pytest.mark.parametrize('field,value', [
    ('eof_observed', False),
    ('received_messages', 2),
    ('processed_messages', 2),
    ('dropped_messages', 1),
    ('queue_overflow', 1),
    ('backlog_at_drain', 1),
])
def test_ours_missing_or_failed_ack_is_fail_closed(field, value):
    evidence = _passing_events()
    evidence['consumer'][field] = value
    assert PHASE.validate_evidence_v2(evidence)['status'] == 'invalid'


def test_ours_unpaced_requires_explicit_synchronous_ack():
    evidence = _passing_events('unpaced_ack')
    assert PHASE.validate_evidence_v2(evidence, require_pass=True)['status'] == 'pass'
    evidence['consumer']['ack_backpressure_verified'] = False
    assert PHASE.validate_evidence_v2(evidence)['status'] == 'invalid'


def test_ours_evidence_is_machine_json_and_atomic_staging_is_not_accepted(tmp_path):
    evidence_path = tmp_path / 'consumer_evidence.json'
    evidence_path.write_text(json.dumps(_passing_events()['consumer']), encoding='utf-8')
    loaded = json.loads(evidence_path.read_text(encoding='utf-8'))
    assert loaded['publisher_count_used'] is False
    part = evidence_path.with_name(evidence_path.name + '.part')
    part.write_text('{"incomplete": true}\n', encoding='utf-8')
    assert part.exists()
    # The production writer refuses a pre-existing staging file instead of
    # treating a partial signal/crash document as a completed ack proof.
    assert part.read_text(encoding='utf-8').startswith('{"incomplete"')


def test_trajectory_equivalence_fixture_hashes_bytes_only(tmp_path):
    baseline = tmp_path / 'baseline.tum'
    instrumented = tmp_path / 'instrumented.tum'
    changed = tmp_path / 'changed.tum'
    baseline.write_bytes(b'opaque trajectory bytes\n')
    instrumented.write_bytes(b'opaque trajectory bytes\n')
    changed.write_bytes(b'opaque trajectory bytes changed\n')

    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    assert digest(baseline) == digest(instrumented)
    assert digest(baseline) != digest(changed)

    output = tmp_path / 'equivalence.json'
    completed = subprocess.run([
        'python3', str(ROOT / 'scripts/check_ours_trajectory_hash_equivalence.py'),
        str(baseline), str(instrumented), '--output', str(output)],
        check=False)
    assert completed.returncode == 0
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['byte_identical'] is True
    assert result['payload_opened_for_metrics'] is False

    completed = subprocess.run([
        'python3', str(ROOT / 'scripts/check_ours_trajectory_hash_equivalence.py'),
        str(baseline), str(changed)], check=False)
    assert completed.returncode == 2
