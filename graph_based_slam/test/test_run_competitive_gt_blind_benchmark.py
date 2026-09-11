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

"""Contract tests for the GT-blind M6a driver."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_competitive_gt_blind_benchmark',
    ROOT / 'scripts/run_competitive_gt_blind_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RUNNER)


def test_schedule_is_explicit_27_attempt_product():
    attempts = RUNNER.schedule()
    assert len(attempts) == 27
    assert [(item['system'], item['slot'], item['repetition'])
            for item in attempts[:3]] == [
                ('ours', 'fresh_1', 1), ('ours', 'fresh_1', 2),
                ('ours', 'fresh_1', 3)]
    assert attempts[-1] == {
        'schedule_index': 27, 'system': 'fast_livo2',
        'slot': 'fresh_3', 'repetition': 3}
    assert RUNNER.schedule_sha(attempts) == RUNNER.schedule_sha()


def test_preflight_hashes_repeated_slot_once(tmp_path, monkeypatch):
    raw = tmp_path / 'exp14.bag'
    raw.write_bytes(b'raw fixture')
    canonical = tmp_path / 'canonical'
    canonical.mkdir()
    (canonical / 'metadata.yaml').write_text('fixture\n')
    item = {
        'slot': 'fresh_1', 'raw_path': raw, 'raw_bytes': raw.stat().st_size,
        'raw_sha256': 'raw-sha', 'canonical_path': canonical,
        'canonical_tree_sha256': 'canonical-sha',
    }
    calls = {'raw': 0, 'canonical': 0}

    def fake_file(path):
        calls['raw'] += 1
        assert path == raw
        return 'raw-sha'

    def fake_tree(path):
        calls['canonical'] += 1
        assert path == canonical
        return 'canonical-sha'

    monkeypatch.setattr(RUNNER, 'sha256_file', fake_file)
    monkeypatch.setattr(RUNNER, 'sha256_tree', fake_tree)
    checked = set()
    RUNNER.verify_input_identity(item, checked)
    RUNNER.verify_input_identity(item, checked)
    assert checked == {'fresh_1'}
    assert calls == {'raw': 1, 'canonical': 1}


def test_input_and_output_roots_must_not_overlap(tmp_path):
    input_root = tmp_path / 'inputs'
    input_root.mkdir()
    with pytest.raises(RUNNER.ContractError):
        RUNNER.assert_roots_are_disjoint(input_root, input_root / 'results')
    with pytest.raises(RUNNER.ContractError):
        nested = input_root / 'nested'
        nested.mkdir()
        RUNNER.assert_roots_are_disjoint(nested, input_root)


def test_gt_path_cannot_be_reached_by_mount_or_environment(tmp_path):
    gt = tmp_path / 'ground_truth.txt'
    gt.write_text('opaque fixture\n')
    bag = tmp_path / 'bag'
    bag.mkdir()
    item = {'gt_realpath': gt.resolve(), 'gt_device': gt.stat().st_dev,
            'gt_inode': gt.stat().st_ino}
    guard = RUNNER._guard_gt(item, [(bag, '/input/bag', 'ro')],
                             ['docker', 'run', '--network', 'none'],
                             {'BAG_PATH': '/input/bag'})
    assert guard['ground_truth_reachable'] is False
    with pytest.raises(RUNNER.ContractError):
        RUNNER._guard_gt(item, [(tmp_path, '/input', 'ro')],
                         ['docker', 'run'], {})
    with pytest.raises(RUNNER.ContractError):
        RUNNER._guard_gt(item, [(bag, '/input/bag', 'ro')],
                         ['docker', 'run'], {'INPUT': 'ground_truth.txt'})


def test_marker_mismatch_and_unowned_root_are_rejected(tmp_path):
    root = tmp_path / 'results'
    root.mkdir()
    identity = {'schedule_sha256': 'a' * 64}
    RUNNER.check_or_create_marker(root, identity, create=True)
    with pytest.raises(RUNNER.ContractError):
        RUNNER.check_or_create_marker(root, {'schedule_sha256': 'b' * 64}, create=False)
    other = tmp_path / 'other'
    other.mkdir()
    (other / 'stale.txt').write_text('stale\n')
    with pytest.raises(RUNNER.ContractError):
        RUNNER.check_or_create_marker(other, identity, create=False)


def test_image_digest_must_be_immutable():
    receipt = {'systems': {'ours': {'container': {
        'image_tag': 'ours:test', 'image_digest': 'not-a-digest'}}}}
    with pytest.raises(RUNNER.ContractError):
        RUNNER.image_ref_and_labels('ours', receipt, inspect=False)


def test_fast_command_has_raw_only_mount_and_semantic_identity():
    item = {
        'raw_path': Path('/managed/slots/exp14/source/exp14.bag'),
        'canonical_path': Path('/managed/slots/exp14/canonical_ros2'),
    }
    command, env = RUNNER.docker_command(
        'fast_livo2', item, 'fast:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 1, 'system': 'fast_livo2',
         'slot': 'fresh_1', 'repetition': 1})
    command_text = '\0'.join(command + list(env.values()))
    assert '/input/raw_input.bag' in command_text
    assert '/input/canonical_ros2' not in command_text
    assert '/ground_truth' not in command_text
    assert '/calibration' not in command_text


def test_fast_command_is_loopback_only_under_network_none():
    item = {
        'raw_path': Path('/managed/slots/exp14/source/exp14.bag'),
        'canonical_path': Path('/managed/slots/exp14/canonical_ros2'),
    }
    command, env = RUNNER.docker_command(
        'fast_livo2', item, 'fast:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 19, 'system': 'fast_livo2',
         'slot': 'fresh_1', 'repetition': 1})
    assert command[command.index('--network') + 1] == 'none'
    assert '--network=host' not in command
    assert env['ROS_MASTER_URI'] == 'http://127.0.0.1:11311'
    assert env['ROS_IP'] == '127.0.0.1'
    assert env['ROS_HOSTNAME'] == '127.0.0.1'


def test_docker_command_leaves_cgroup_memory_uncapped_for_process_rss():
    item = {
        'raw_path': Path('/managed/slots/exp14/source/exp14.bag'),
        'canonical_path': Path('/managed/slots/exp14/canonical_ros2'),
    }
    command, _ = RUNNER.docker_command(
        'ours', item, 'ours:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 1, 'system': 'ours',
         'slot': 'fresh_1', 'repetition': 1})
    assert '--memory' not in command
    assert '--memory-swap' not in command


def test_v2_phase_selection_is_explicit_and_does_not_fake_unpaced_ack():
    item = {
        'raw_path': Path('/managed/slots/exp14/source/exp14.bag'),
        'canonical_path': Path('/managed/slots/exp14/canonical_ros2'),
    }
    command, env = RUNNER.docker_command(
        'ours', item, 'ours:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 1, 'system': 'ours',
         'slot': 'fresh_1', 'repetition': 1}, phase_contract='v2-paced')
    assert env['M6A10_PHASE_CONTRACT_VERSION'] == \
        'm6a10-online-compute-v2'
    assert env['M6A10_PHASE_MODE'] == 'paced_1x'
    assert 'M6A10_ACK_BACKPRESSURE_ENABLED' not in env
    command, env = RUNNER.docker_command(
        'ours', item, 'ours:test@sha256:' + 'a' * 64,
        Path('/M6A_OUTPUT_PLACEHOLDER'),
        {'schedule_index': 2, 'system': 'ours',
         'slot': 'fresh_1', 'repetition': 1}, phase_contract='v2-unpaced')
    assert env['M6A10_PHASE_MODE'] == 'unpaced_ack'
    assert 'M6A10_ACK_BACKPRESSURE_ENABLED' not in env
    assert '--network' in command and command[command.index('--network') + 1] == 'none'


def test_campaign4_plan_binds_m6a7_process_rss_contract():
    receipt_path = ROOT / 'configs' / 'slam_benchmark_profiles' / (
        'competitive_execution_selection_2026-08.yaml')
    receipt = RUNNER.yaml.safe_load(receipt_path.read_text(encoding='utf-8'))
    # The retained receipt points at historical external M6a7 evidence.  The
    # current workspace cannot reopen that volume, so the production helper
    # must fail closed rather than treating absent bytes as a PASS.
    with pytest.raises(RUNNER.ContractError, match='path/SHA does not match'):
        RUNNER.m6a7_contract_identity(receipt)


def test_runtime_writable_state_is_scoped_to_attempt_output():
    glim = (ROOT / 'scripts' / 'glim_container_run.sh').read_text()
    fast = (ROOT / 'scripts' / 'fast_livo2_container_run.sh').read_text()
    assert 'ROS_HOME="${ROS_HOME:-${OUT_DIR}/ros_home}"' in glim
    assert 'ROS_LOG_DIR="${ROS_LOG_DIR:-${OUT_DIR}/ros_log}"' in glim
    assert 'mkdir -p "/root/.ros' not in glim
    assert 'export ROS_IP=127.0.0.1' in fast
    assert 'export ROS_HOSTNAME=127.0.0.1' in fast


def test_frozen_manifest_semantic_mismatch_is_rejected(tmp_path):
    root = tmp_path / 'managed'
    canonical = root / 'slots/exp14/canonical_ros2'
    source = root / 'slots/exp14/source'
    canonical.mkdir(parents=True)
    source.mkdir(parents=True)
    (canonical / 'metadata.yaml').write_text('metadata\n')
    raw = source / 'exp14.bag'
    raw.write_bytes(b'raw')
    gt = root / 'slots/exp14/ground_truth/exp14.txt'
    gt.parent.mkdir()
    gt.write_bytes(b'opaque')
    manifest = {
        'status': 'frozen_unopened',
        'raw_bag': {'sha256': 'a' * 64},
        'canonical_rosbag2': {
            'canonical_rosbag2_tree_sha256': 'b' * 64,
            'semantic_equivalence_sha256': 'c' * 64},
        'semantic_equivalence_sha256': 'c' * 64,
        'input_manifest_sha256': 'd' * 64,
        'calibration': {'sha256': 'e' * 64},
    }
    manifest_path = root / 'slots/exp14/manifest.json'
    manifest_path.write_text(json.dumps(manifest))
    identity = {
        'raw_bag_path': 'slots/exp14/source/exp14.bag',
        'ground_truth_path': 'slots/exp14/ground_truth/exp14.txt',
        'manifest_path': 'slots/exp14/manifest.json',
        'manifest_file_sha256': RUNNER.sha256_file(manifest_path),
        'raw_bag_sha256': 'a' * 64, 'raw_bag_bytes': 3,
        'canonical_rosbag2_tree_sha256': 'b' * 64,
        'semantic_equivalence_sha256': 'c' * 64,
        'input_manifest_sha256': 'd' * 64,
        'calibration_archive_sha256': 'e' * 64,
    }
    profile = {'datasets': {'fresh_holdout_slots': {
        'fresh_1': {'status': 'frozen_unopened', 'sequence': 'exp14',
                    'frozen_identity': identity}}}}
    manifest['semantic_equivalence_sha256'] = 'f' * 64
    manifest['canonical_rosbag2']['semantic_equivalence_sha256'] = 'f' * 64
    manifest_path.write_text(json.dumps(manifest))
    identity['manifest_file_sha256'] = RUNNER.sha256_file(manifest_path)
    with pytest.raises(RUNNER.ContractError, match='semantic'):
        RUNNER.resolve_slot(root, profile, 'fresh_1')


def test_nonzero_timeout_is_retained_as_immutable_failed_attempt(tmp_path, monkeypatch):
    output = tmp_path / 'results'
    output.mkdir()
    item = {
        'schedule_index': 1, 'system': 'glim_cpu', 'slot': 'fresh_1',
        'sequence': 'exp14', 'repetition': 1,
        'image_digest': 'sha256:' + 'a' * 64,
        'execution_identity': {'revision': 'b' * 40},
        'input': {'semantic_equivalence_sha256': 'c' * 64},
        'profile_canonical_sha256': 'd' * 64,
        'execution_receipt_file_sha256': 'e' * 64,
        'selection_receipt_file_sha256': 'f' * 64,
        'argv': ['docker', 'run', '-v', '/M6A_OUTPUT_PLACEHOLDER:/out'],
        'env': {'OMP_NUM_THREADS': '8'},
        'mounts': [{'source': '/managed/bag', 'target': '/input/bag', 'mode': 'ro'}],
        'gt_blind_guard': {'ground_truth_reachable': False,
                           'gt_device': 1, 'gt_inode': 2,
                           'mount_sources': ['/managed/bag']},
    }

    # The driver catches subprocess.TimeoutExpired; use that exact exception
    # to exercise the formal timeout path rather than a generic OS failure.
    def expired(*_args, **_kwargs):
        raise RUNNER.subprocess.TimeoutExpired(['docker', 'run'], 1)

    monkeypatch.setattr(RUNNER.subprocess, 'run', expired)
    report = RUNNER.run_attempt(item, output, 1.0)
    final = output / 'attempt_001'
    assert final.is_dir() and not (output / 'attempt_001.part').exists()
    assert report['execution']['timed_out'] is True
    assert report['completion']['complete'] is False
    assert json.loads((final / 'attempt.json').read_text())['execution']['exit_status'] == 124


def test_nonzero_and_missing_output_are_retained_as_failed_attempt(tmp_path, monkeypatch):
    output = tmp_path / 'results'
    output.mkdir()
    item = {
        'schedule_index': 2, 'system': 'ours', 'slot': 'fresh_1',
        'sequence': 'exp14', 'repetition': 2,
        'image_digest': 'sha256:' + 'a' * 64,
        'execution_identity': {'revision': 'b' * 40},
        'input': {'semantic_equivalence_sha256': 'c' * 64},
        'profile_canonical_sha256': 'd' * 64,
        'execution_receipt_file_sha256': 'e' * 64,
        'selection_receipt_file_sha256': 'f' * 64,
        'argv': ['docker', 'run', '-v', '/M6A_OUTPUT_PLACEHOLDER:/out'],
        'env': {'OMP_NUM_THREADS': '8'},
        'mounts': [{'source': '/managed/bag', 'target': '/input/bag', 'mode': 'ro'}],
        'gt_blind_guard': {'ground_truth_reachable': False,
                           'gt_device': 1, 'gt_inode': 2,
                           'mount_sources': ['/managed/bag']},
    }

    class Completed:
        returncode = 17

    monkeypatch.setattr(RUNNER.subprocess, 'run', lambda *_args, **_kwargs: Completed())
    report = RUNNER.run_attempt(item, output, 1.0)
    final = output / 'attempt_002'
    assert final.is_dir() and report['completion']['complete'] is False
    assert json.loads((final / 'attempt.json').read_text())['execution']['exit_status'] == 17


def test_existing_attempt_cannot_be_overwritten(tmp_path):
    output = tmp_path / 'results'
    output.mkdir()
    (output / 'attempt_001').mkdir()
    item = {'schedule_index': 1, 'system': 'ours', 'slot': 'fresh_1',
            'sequence': 'exp14', 'repetition': 1}
    with pytest.raises(RUNNER.ContractError, match='already exists'):
        RUNNER.run_attempt(item, output, 1.0)


def test_expected_output_contract_is_fail_closed():
    assert RUNNER.expected_outputs('ours', Path('/out')) == [
        Path('/out/traj_raw.tum'), Path('/out/container_memory.json'),
        Path('/out/container_process_rss.json'), Path('/out/phase_evidence.json')]


def test_phase_evidence_is_primary_and_missing_is_invalid(tmp_path):
    assert RUNNER.parse_phase_evidence(tmp_path / 'missing.json')['valid'] is False
    value = {
        'schema_version': 1,
        'contract_version': 'm6a10-online-compute-v1',
        'events_monotonic_ns': {
            name: (index + 1) * 1_000_000_000
            for index, name in enumerate(
                ('startup_start', 'startup_end', 'input_start', 'input_end',
                 'drain_start', 'drain_end', 'postprocess_start',
                 'postprocess_end', 'save_start', 'save_end',
                 'shutdown_start', 'shutdown_end'))},
        'input_duration_seconds': 10.0,
        'maximum_trajectory_end_gap_seconds': 0.25,
        'coverage': {
            'status': 'verified', 'mode': 'trajectory_timestamp_coverage',
            'complete': True, 'dropped_messages': 0, 'queue_overflow': 0,
            'last_input_timestamp_seconds': 109.9,
            'required_end_timestamp_seconds': 110.0,
            'end_gap_seconds': 0.1,
        },
        'resource': {
            'status': 'verified', 'cpu_user_seconds': 1.0,
            'cpu_system_seconds': 1.0, 'io_input_operations': 1,
            'io_output_operations': 1,
        },
        'exit_status': 0,
    }
    path = tmp_path / 'phase_evidence.json'
    path.write_text(json.dumps(value), encoding='utf-8')
    checked = RUNNER.parse_phase_evidence(path)
    assert checked['valid'] is True
    assert checked['runtime']['online_compute_rtf'] == pytest.approx(0.3)
    mismatched = RUNNER.parse_phase_evidence(
        path, expected_contract_version=RUNNER.PHASE_CONTRACT_VERSION_V2,
        expected_phase_mode='paced_1x')
    assert mismatched['valid'] is False
    assert 'does not match scheduled mode' in mismatched['reason']
    value['coverage']['dropped_messages'] = 1
    path.write_text(json.dumps(value), encoding='utf-8')
    assert RUNNER.parse_phase_evidence(path)['valid'] is False
    assert RUNNER.expected_outputs('glim_cpu', Path('/out')) == [
        Path('/out/dump/traj_lidar.txt'), Path('/out/container_memory.json'),
        Path('/out/container_process_rss.json'), Path('/out/phase_evidence.json')]
    assert RUNNER.expected_outputs('fast_livo2', Path('/out')) == [
        Path('/out/odometry.csv'), Path('/out/container_memory.json'),
        Path('/out/container_process_rss.json'), Path('/out/phase_evidence.json')]


def test_v2_ours_requires_application_consumer_evidence():
    assert RUNNER.expected_outputs(
        'ours', Path('/out'), RUNNER.PHASE_CONTRACT_VERSION_V2) == [
            Path('/out/traj_raw.tum'), Path('/out/container_memory.json'),
            Path('/out/container_process_rss.json'),
            Path('/out/phase_evidence.json'),
            Path('/out/consumer_evidence.json')]


def _valid_process_rss_evidence():
    return {
        'schema_version': 1,
        'measurement_version': 'm6a7-container-process-rss-v1',
        'measurement_scope': 'container_pid_namespace_proc_status',
        'primary_metric': 'aggregate_process_tree_peak_rss_bytes',
        'primary_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'status': 'pass',
        'atomic': True,
        'sampler_excluded': True,
        'sampler_pid': 123,
        'first_sample_monotonic_ns': 1,
        'last_sample_monotonic_ns': 2,
        'sample_count': 3,
        'sample_errors': 0,
        'pid_race_skips': 0,
        'missed_intervals': 0,
        'interval_jitter_percent': 1.0,
        'thresholds': {'min_samples': 2, 'max_errors': 0,
                       'max_race_skips': 0, 'max_jitter_percent': 10.0},
        'peak': {
            'vmrss_bytes': 140,
            'rss_anon_bytes': 100,
            'rss_file_bytes': 30,
            'rss_shmem_bytes': 10,
            'process_count': 2,
        },
        'aggregate_process_tree_peak_rss_bytes': 140,
    }


def _valid_memory_evidence():
    process = _valid_process_rss_evidence()
    pressure = {
        'some': {'avg10': 0.0, 'avg60': 0.0, 'avg300': 0.0, 'total': 0},
        'full': {'avg10': 0.0, 'avg60': 0.0, 'avg300': 0.0, 'total': 0},
    }
    return {
        'schema_version': 2,
        'measurement_version': 'm6a7-container-memory-v2',
        'status': 'pass',
        'measurement_scope': 'container_cgroup_v2_with_pid_rss',
        'children_included': True,
        'cgroup_version': 2,
        'cgroup_path': '/docker/test',
        'proc_self_cgroup': '0::/docker/test',
        'container_cgroup_peak_bytes': 140,
        'memory_current_bytes': 100,
        'memory_max_raw': '512',
        'memory_max_bytes': 512,
        'memory_max_unlimited': False,
        'container_cgroup_total_peak_bytes': 140,
        'primary_metric': 'aggregate_process_tree_peak_rss_bytes',
        'primary_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'process_rss_metric_definition':
            'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted',
        'aggregate_process_tree_peak_rss_bytes': 140,
        'process_tree_peak_rss_bytes': 140,
        'process_rss_evidence_path': '/out/container_process_rss.json',
        'process_rss_evidence': process,
        'cgroup_events': {
            'status': 'pass', 'oom_free': True,
            'memory_events': {
                'baseline': {'oom': 0, 'oom_kill': 0},
                'final': {'oom': 0, 'oom_kill': 0},
                'delta': {'oom': 0, 'oom_kill': 0}},
            'memory_events_local': {
                'baseline': {'oom': 0, 'oom_kill': 0},
                'final': {'oom': 0, 'oom_kill': 0},
                'delta': {'oom': 0, 'oom_kill': 0}},
            'memory_pressure': {
                'baseline': pressure, 'final': pressure, 'delta': pressure},
            'oom_delta': {'memory_events.oom': 0,
                          'memory_events.oom_kill': 0,
                          'memory_events_local.oom': 0,
                          'memory_events_local.oom_kill': 0},
        },
        'atomic': True,
        'output_readability': {'status': 'pass'},
    }


def test_container_memory_evidence_requires_numeric_cgroup_v2_values(tmp_path):
    path = tmp_path / 'container_memory.json'
    (tmp_path / 'container_process_rss.json').write_text(
        json.dumps(_valid_process_rss_evidence()))
    path.write_text(json.dumps(_valid_memory_evidence()))
    valid = RUNNER.parse_container_memory_evidence(path)
    assert valid['valid'] is True
    assert valid['container_cgroup_peak_bytes'] == 140
    for field, value in (
            ('container_cgroup_peak_bytes', None),
            ('memory_current_bytes', 'not-a-number')):
        document = _valid_memory_evidence()
        document[field] = value
        path.write_text(json.dumps(document))
        invalid = RUNNER.parse_container_memory_evidence(path)
        assert invalid['valid'] is False
    unlimited = _valid_memory_evidence()
    unlimited['memory_max_raw'] = 'max'
    unlimited['memory_max_bytes'] = None
    unlimited['memory_max_unlimited'] = True
    path.write_text(json.dumps(unlimited))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is True
    inconsistent = dict(unlimited)
    inconsistent['memory_max_unlimited'] = False
    path.write_text(json.dumps(inconsistent))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    over_max = _valid_memory_evidence()
    over_max['memory_current_bytes'] = 513
    path.write_text(json.dumps(over_max))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False


def test_container_memory_evidence_missing_and_unreadable_are_invalid(tmp_path):
    path = tmp_path / 'container_memory.json'
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    document = _valid_memory_evidence()
    document['output_readability'] = {'status': 'invalid', 'scope': 'runner'}
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    path.write_bytes(b'{"status":\xff}')
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False


def test_container_memory_evidence_rejects_oom_and_process_rss_mismatch(tmp_path):
    path = tmp_path / 'container_memory.json'
    sampler_path = tmp_path / 'container_process_rss.json'
    sampler_path.write_text(json.dumps(_valid_process_rss_evidence()))
    document = _valid_memory_evidence()
    document['cgroup_events']['oom_free'] = False
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    document = _valid_memory_evidence()
    document['process_tree_peak_rss_bytes'] = 141
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    document = _valid_memory_evidence()
    document.pop('primary_metric')
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    document = _valid_memory_evidence()
    document['aggregate_process_tree_peak_rss_bytes'] = 141
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False
    document = _valid_memory_evidence()
    document['cgroup_events']['memory_pressure']['final']['some']['total'] = 'bad'
    path.write_text(json.dumps(document))
    assert RUNNER.parse_container_memory_evidence(path)['valid'] is False


def test_process_rss_evidence_rejects_missing_samples_and_client_only_metric(tmp_path):
    path = tmp_path / 'container_process_rss.json'
    document = _valid_process_rss_evidence()
    document['sample_count'] = 1
    path.write_text(json.dumps(document))
    assert RUNNER.parse_process_rss_evidence(path)['valid'] is False
    document = _valid_process_rss_evidence()
    document['peak'].pop('vmrss_bytes')
    path.write_text(json.dumps(document))
    assert RUNNER.parse_process_rss_evidence(path)['valid'] is False
    document = _valid_process_rss_evidence()
    document['sample_errors'] = 1
    path.write_text(json.dumps(document))
    assert RUNNER.parse_process_rss_evidence(path)['valid'] is False


def test_client_rss_is_not_the_container_memory_metric(tmp_path):
    path = tmp_path / 'host_time.txt'
    path.write_text(
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 0:01.00\n'
        'Maximum resident set size (kbytes): 29116\n')
    timing = RUNNER.parse_time_report(path)
    assert timing['docker_client_peak_rss_kb'] == 29116
    assert 'peak_rss_kb' not in timing
    assert RUNNER.comparison_rss_bytes({'process_tree_peak_rss_bytes': 4096}) == 4096
    assert RUNNER.comparison_rss_bytes(
        {'aggregate_process_tree_peak_rss_bytes': 4096}) == 4096
    with pytest.raises(RUNNER.ContractError):
        RUNNER.comparison_rss_bytes({'docker_client_peak_rss_kb': 29116})


def test_permission_during_finalization_keeps_part_and_writes_failure(
        tmp_path, monkeypatch):
    output = tmp_path / 'results'
    output.mkdir()
    item = {
        'schedule_index': 7, 'system': 'ours', 'slot': 'fresh_1',
        'sequence': 'exp14', 'repetition': 1,
        'image_digest': 'sha256:' + 'a' * 64,
        'execution_identity': {'revision': 'b' * 40},
        'input': {'semantic_equivalence_sha256': 'c' * 64},
        'profile_canonical_sha256': 'd' * 64,
        'execution_receipt_file_sha256': 'e' * 64,
        'selection_receipt_file_sha256': 'f' * 64,
        'argv': ['docker', 'run', '-v', '/M6A_OUTPUT_PLACEHOLDER:/out'],
        'env': {'OMP_NUM_THREADS': '8'},
        'mounts': [{'source': '/managed/bag', 'target': '/input/bag',
                    'mode': 'ro'}],
        'gt_blind_guard': {'ground_truth_reachable': False,
                           'gt_device': 1, 'gt_inode': 2,
                           'mount_sources': ['/managed/bag']},
    }

    class Completed:
        returncode = 0

    monkeypatch.setattr(RUNNER.subprocess, 'run', lambda *_a, **_k: Completed())
    monkeypatch.setattr(RUNNER, 'output_tree_hash',
                        lambda _path: (_ for _ in ()).throw(
                            PermissionError('fixture unreadable')))
    with pytest.raises(RUNNER.ContractError, match='finalization failed'):
        RUNNER.run_attempt(item, output, 1.0)
    assert (output / 'attempt_007.part').is_dir()
    failure = output / 'attempt_007.driver_failure.json'
    assert failure.is_file()
    document = json.loads(failure.read_text())
    assert document['preserve_part'] is True
    assert document['attempt_finalized'] is False


def test_gt_blind_wrappers_require_explicit_mode():
    for name in ('ours_container_gt_blind_run.sh',
                 'glim_container_gt_blind_run.sh',
                 'fast_livo2_container_gt_blind_run.sh'):
        text = (ROOT / 'scripts' / name).read_text()
        if name.startswith('ours_'):
            assert '--gt-blind' in text
            assert 'BAG_PATH is required' in text
        else:
            assert 'GT_BLIND' in text
            assert 'GT_BLIND=1 is required' in text
