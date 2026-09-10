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


"""Contract tests for the frozen GLIM and FAST-LIVO2 competition profile."""

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
PROFILE_PATH = (
    ROOT / 'configs' / 'slam_benchmark_profiles' / 'competitive_slam_v1.yaml'
)
HILTI_RKO_PATH = ROOT / 'configs' / 'hilti2022' / 'rko_lio_hilti2022_pandar.yaml'
EXECUTION_RECEIPT_PATH = ROOT / 'configs' / 'slam_benchmark_profiles' / (
    'competitive_execution_selection_2026-08.yaml')
EXECUTION_CHECKER_PATH = ROOT / 'scripts' / 'check_competitive_execution_selection.py'
_CHECKER_SPEC = importlib.util.spec_from_file_location(
    'competitive_execution_selection_checker', EXECUTION_CHECKER_PATH)
assert _CHECKER_SPEC.loader is not None
_CHECKER = importlib.util.module_from_spec(_CHECKER_SPEC)
_CHECKER_SPEC.loader.exec_module(_CHECKER)


def _profile():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    return document['competitive_slam_profile']


def test_rivals_are_pinned_to_full_git_revisions():
    profile = _profile()
    for rival in profile['rivals'].values():
        assert len(rival['revision']) == 40
        int(rival['revision'], 16)
    assert len(profile['rivals']['glim']['ros2_revision']) == 40
    int(profile['rivals']['glim']['ros2_revision'], 16)
    assert profile['rivals']['glim']['revision'] != profile['rivals']['glim'][
        'annotated_tag_object']


def test_win_policy_requires_three_real_repeated_holdouts():
    profile = _profile()
    policy = profile['win_policy']
    assert profile['repetitions'] == 3
    assert policy['minimum_assigned_holdouts'] >= 3
    assert policy['minimum_holdout_wins'] >= 3
    assert policy['minimum_primary_improvement_percent'] == 3.0
    assert policy['maximum_realtime_factor'] <= 1.0
    assert policy['maximum_peak_rss_ratio_to_rival'] <= 1.2
    assert policy['maximum_mapping_regression_percent'] <= 2.0
    assert policy['maximum_visual_colour_regression_percent'] <= 2.0


def test_seen_datasets_cannot_silently_become_holdouts():
    profile = _profile()
    datasets = profile['datasets']
    seen = set(datasets['bringup']) | set(datasets['development'])
    seen |= set(datasets['regression_only'])
    holdouts = datasets['holdout_slots']
    assert not seen.intersection(holdouts)
    assert len({slot['dataset'] for slot in holdouts.values()}) == len(holdouts)
    assert all(
        slot['status'] in {'assigned_inputs_pending_hash', 'frozen'}
        for slot in holdouts.values()
    )
    assert all(slot['bag_expected_bytes'] > 0 for slot in holdouts.values())
    assert all(slot['bag_url'].startswith('https://') for slot in holdouts.values())
    assert all(
        slot['ground_truth_url'].startswith('https://')
        for slot in holdouts.values()
    )
    assert all(len(slot['ground_truth_sha256']) == 64 for slot in holdouts.values())
    assert all(
        len(slot['calibration_archive_sha256']) == 64
        for slot in holdouts.values()
    )
    assert all(slot['status'] == 'frozen' for slot in holdouts.values())
    for slot in holdouts.values():
        for key in ('raw_rosbag1_sha256', 'canonical_rosbag2_tree_sha256',
                    'input_manifest_sha256', 'semantic_equivalence_sha256'):
            assert len(slot[key]) == 64
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_slots_assigned'
    ] is True
    assert profile['phase_gates']['before_algorithm_tuning'][
        'require_all_holdout_inputs_frozen'
    ] is True


def test_fresh_v2_slots_are_deep_verified_but_unscored():
    profile = _profile()
    slots = profile['datasets']['fresh_holdout_slots']
    assert len(slots) >= 3
    assert all(slot['status'] == 'frozen_unopened' for slot in slots.values())
    assert [slot['sequence'] for slot in slots.values()] == ['exp14', 'exp16', 'exp18']
    assert all(len(slot['selection_receipt_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['input_manifest_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['ground_truth_sha256']) == 64 for slot in slots.values())
    assert all(len(slot['calibration_archive_sha256']) == 64 for slot in slots.values())
    receipt_path = ROOT / slots['fresh_1']['selection_receipt_path']
    assert receipt_path.is_file()
    receipt = yaml.safe_load(receipt_path.read_text(encoding='utf-8'))
    assert receipt['status'] == 'frozen_unopened'
    assert receipt['selection_decision']['no_performance_data_used'] is True
    assert receipt['selection_decision']['no_ground_truth_content_opened'] is True
    exposed = profile['datasets']['holdout_slots']
    assert not set(slots).intersection(exposed)


def test_execution_selection_receipt_is_registered_and_ready_preflight():
    profile = _profile()
    policy = profile['evidence_gate_v2']
    assert policy['require_execution_selection_receipt'] is True
    path = ROOT / policy['execution_selection_receipt_path']
    assert path == EXECUTION_RECEIPT_PATH
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        policy['execution_selection_receipt_sha256'])
    receipt = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert receipt['receipt_kind'] == 'competitive_execution_selection'
    assert receipt['status'] == 'ready'
    assert receipt['systems']['ours']['repository']['revision_status'] == 'pinned'
    assert receipt['systems']['ours']['repository']['worktree_dirty'] is False
    assert receipt['systems']['ours']['repository']['revision'] == (
        '866f733677e92ecb08d67126e463da99dd140d46')
    assert receipt['common_identity']['machine_fingerprint']['status'] == 'ready'
    assert receipt['common_identity']['thread_policy']['status'] == 'ready'
    assert receipt['common_identity']['thread_policy']['cpu_affinity'] == list(range(8))
    assert receipt['common_identity']['thread_policy']['canonical_sha256'] == (
        '262d656a7c382cd27696b3150215ae563a014796de69718754858bcb814ba993')
    enforcement = receipt['common_identity']['thread_policy']['enforcement']
    assert 'taskset' in enforcement['cpu_affinity']
    assert 'docker_cpuset' in enforcement['cpu_affinity']
    assert enforcement['required_before_run'] is True


def test_m6a10_materializer_uses_only_ntu_tnp01_semantic_topics():
    profile = _profile()
    contract = profile['runtime_policy']['phase_contract_v2'][
        'synchronized_tail_materialization']
    expected_topics = [
        '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw']
    semantic = contract['ros1_fast_livo2']['semantic_comparator']
    assert semantic['topics'] == expected_topics
    assert not any(
        topic.startswith('/hesai/') or topic.startswith('/alphasense/')
        for topic in semantic['topics'])
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    selection_semantic = selection['m6a10_phase_contract']['v2_additive_contract'][
        'synchronized_tail_materialization']['ros1_fast_livo2'][
        'semantic_comparator']
    assert selection_semantic['topics'] == expected_topics
    assert selection_semantic['topics'] == semantic['topics']
    assert contract['output_policy']['order_definition'] == (
        'anyreader_chronological_playback_order_including_deterministic_ties')


def test_m6a10_fixed10_materialization_and_ros1_identity_are_preregistered():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'synchronized_tail_materialization']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'synchronized_tail_materialization']
    assert profile_contract == selection_contract
    assert profile_contract['contract_id'] == (
        'm6a10-v2a-synchronized-tail-materialization-v1')
    assert profile_contract['status'] == 'ros2_materialized_verified_ros1_verified'
    assert profile_contract['producer'] == {
        'path': 'scripts/materialize_m6a10_synchronized_tail.py',
        'sha256': '2a2ff6476c6996a20b3e27d375ef33528ad08f20ea8b9bb539c8359075a5ee2b',
    }
    assert profile_contract['contract_test'] == {
        'path': 'graph_based_slam/test/test_materialize_m6a10_synchronized_tail.py',
        'sha256': 'f0c58ddbdfeaae2399125bfffc0ebef5ef5c57aba4dca72e7e8238ec0c3175d4',
    }

    output = profile_contract['generated_output']
    assert output['path'].endswith('tnp_01_m6a10_v2a_sync_materialization_v1_ros2')
    assert output['tree_sha256'] == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
    assert output['tree_hash_kind'] == 'relative_path_size_content_sha256_v1'
    assert output['input_record_count'] == 236688
    assert output['output_record_count'] == 236687
    assert output['ordered_payload_stream_sha256'] == (
        '1512e3946d013b8543f09146b81474745ad9e718910b5f1322bda2823546e101')
    assert output['per_topic']['/os1_cloud_node1/points']['count'] == 5793
    assert output['per_topic']['/imu/imu']['count'] == 225102
    assert output['per_topic']['/left/image_raw']['count'] == 5792

    generated_receipt = profile_contract['generated_receipt']
    receipt_path = Path(generated_receipt['path'])
    assert generated_receipt['sha256'] == (
        '62defcf7a1b5cdadee666de44b09f71f8e011551ea93e37eec51b64a333cae9f')
    # This historical receipt is on the unavailable evidence volume in the
    # current workspace.  Preserve the production fail-closed state instead
    # of reading through an empty/unmounted mountpoint or upgrading it.
    if not receipt_path.is_file():
        assert str(receipt_path).startswith('/media/')
        return
    assert receipt_path.is_file()
    assert hashlib.sha256(receipt_path.read_bytes()).hexdigest() == (
        generated_receipt['sha256'])
    assert generated_receipt['contract_id'] == profile_contract['contract_id']
    assert generated_receipt['analyzer_contract_id'] == (
        'm6a10-v2a-synchronized-tail-v1')
    assert generated_receipt['status'] == 'PASS'
    assert generated_receipt['verification']['status'] == 'PASS'
    assert generated_receipt['verification']['method'] == (
        'independent_raw_stream_reread')
    assert generated_receipt['verification']['connection_metadata_equal'] is True
    assert generated_receipt['dropped_lidar']['count'] == 1
    assert generated_receipt['dropped_lidar']['payload_sha256'] == (
        'a11e441e679c424d31a43755d96328f94e73a1d2a52e48a1c808ac51f7443830')
    assert generated_receipt['order_definition'] == (
        'anyreader_chronological_playback_order_including_deterministic_ties')
    assert generated_receipt['safety']['ground_truth_content_opened'] is False
    assert generated_receipt['safety']['scorer_invoked'] is False

    ros1 = profile_contract['ros1_fast_livo2']
    assert ros1['conversion_executed_here'] is True
    assert ros1['status'] == 'ros1_verified'
    assert ros1['destination_extension'] == '.bag'
    assert ros1['staging_policy'] == 'sibling_container_exact_final_basename'
    assert ros1['converter_version'] == '0.11.0'
    assert len(ros1['converter_executable_sha256']) == 64
    assert len(ros1['converter_help_sha256']) == 64
    assert ros1['conversion_argv'][-1] == 'ros1_noetic'
    assert ros1['conversion_argv'][4].endswith(
        '<ros1_staging_container>/<ros1_final_basename>.bag')
    semantic = ros1['semantic_comparator']
    assert semantic['sha256'] == (
        '7464d0e64ba1cafbdbb7a2e162bd2735b7288d9ea6e3a2dc1e7757bdab5fcf41')
    assert semantic['status'] == 'PASS'
    assert semantic['all_topics_equal'] is True
    assert semantic['result_sha256'] == (
        '0a64efb618c9bdbba4e762c350f21e36fb1244b4e2c2ffa17b3d7604cdd318ec')
    assert semantic['topics'] == [
        '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw']
    assert not any(
        topic.startswith('/hesai/') or topic.startswith('/alphasense/')
        for topic in semantic['topics'])

    wrapper = ros1['wrapper']
    assert wrapper['path'] == 'scripts/materialize_m6a10_ros1_equivalent.py'
    assert wrapper['sha256'] == (
        'c018c4d9a80dcb66e62431ea815365ee6812ab69359f845ab321d5dbcf685233')
    assert wrapper['test_path'] == (
        'graph_based_slam/test/test_materialize_m6a10_ros1_equivalent.py')
    assert wrapper['test_sha256'] == (
        'f3572921c330fd0a9b8f504caac6d775938495f244b918f15c8e109dddc720ea')
    assert wrapper['contract_id'] == 'm6a10-v2a-ros1-equivalence-v1'
    assert wrapper['status'] == 'wrapper_executed_verified'
    assert wrapper['evidence']['receipt_path'].endswith(
        '/ros1_equivalence_v1/receipt.json')
    assert wrapper['evidence']['semantic_report_path'].endswith(
        '/ros1_equivalence_v1/semantic.json')
    assert wrapper['evidence']['time_report_path'].endswith(
        '/ros1_equivalence_v1/time-v.txt')
    assert wrapper['result']['status'] == 'PASS'
    assert wrapper['result']['receipt_sha256'] == (
        'b71e7f5aa8f82a5711e6b25e42532caa8248a3fcf062d9b3b15a5c86460438ec')
    assert wrapper['result']['ros1_output_sha256'] == (
        '5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310')
    assert wrapper['result']['ros1_output_bytes'] == 11290464091
    assert wrapper['result']['message_count'] == 236687
    assert wrapper['result']['time_elapsed_wall_seconds'] == 207.57
    assert wrapper['result']['time_max_rss_kib'] == 153252
    assert wrapper['safety']['ground_truth_content_opened'] is False
    assert wrapper['safety']['scorer_invoked'] is False
    assert wrapper['command']['argv'][1] == wrapper['path']
    assert wrapper['command']['argv'][-1].endswith('/semantic.json')

    lineage = profile_contract['superseded_lineage']
    assert lineage['status'] == 'superseded_not_promoted'
    assert lineage['receipt_sha256'] == (
        'c6f0cec83956b3405f841242db0e8d5b71630477701728add74af2b3115c64a9')
    assert lineage['output_tree_sha256'] == (
        '4b4500450306aa60bf4b9daa79dc29a2cae8743a2b5ace197413bd06544fee9e')


def test_m6a10_ours_fixed10_unpaced_replay_contract_is_preregistered():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_replay']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_replay']
    assert profile_contract == selection_contract
    assert profile_contract['status'] == 'preregistered_not_measured'
    assert profile_contract['performance_comparison'] is False
    assert profile_contract['wall_rtf_role'] == 'diagnostic_only'
    assert profile_contract['input']['tree_sha256'] == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926')
    assert profile_contract['input']['expected_consumer_count'] == 230895
    assert profile_contract['input']['eligible_lidar_count'] == 5793
    assert profile_contract['input']['imu_count'] == 225102
    assert profile_contract['input']['image_count'] == 5792
    assert profile_contract['input']['image_consumer_scope'] == (
        'not_consumed_by_ours_direct_visual_frontend_false')
    execution = profile_contract['execution']
    assert execution['image_digest'] == (
        'sha256:2ee632eb0bcde28d74b90ea91975c2b560956cee293c7d36248b0d63fd648329')
    assert execution['algorithm_revision'] == (
        '866f733677e92ecb08d67126e463da99dd140d46')
    assert execution['image_bound_binary'].endswith('/rko_lio/offline_node')
    assert execution['rko_config_sha256'] == (
        '3392a87fa587152e0659f1be6c5121b761a76d5a5e51ba7bc1df641d220cc89f')
    assert profile_contract['command']['phase_mode'] == 'unpaced_ack'
    assert profile_contract['command']['network'] == 'none'
    assert profile_contract['command']['input_mount_read_only'] is True
    consumer = profile_contract['consumer_contract']
    assert consumer['ack_source_kind'] == 'synchronous_processing'
    assert consumer['ack_backpressure_required'] is True
    assert consumer['require_exact_counts'] is True
    assert consumer['require_eof_observed'] is True
    assert consumer['require_empty_drain_backlog'] is True
    assert consumer['require_zero_dropped_messages'] is True
    assert consumer['require_zero_queue_overflow'] is True
    assert consumer['expected_image_messages'] == 0
    assert profile_contract['fixed9_lineage']['old_status'] == (
        'INVALID_DRAIN_TIMEOUT')
    assert profile_contract['fixed9_lineage']['old_terminal_lidar_buffer_size'] == 1
    assert profile_contract['fixed9_lineage']['old_timestamp_gap_ns'] == -15805696
    assert profile_contract['safety']['ground_truth_content_opened'] is False
    assert profile_contract['safety']['scorer_invoked'] is False
    failed = profile_contract['fixed10_v1_attempt']
    assert failed['status'] == 'FAIL_CLOSED'
    assert failed['verification_status'] == 'FAIL_CLOSED'
    assert failed['consumer_status'] == 'PASS'
    assert failed['phase_status'] == 'PASS'
    assert failed['map_artifact_count'] == 18
    assert failed['retries'] == 0
    assert failed['ground_truth_content_opened'] is False
    assert failed['scorer_invoked'] is False


def test_m6a10_fixed10_v2_no_map_contract_is_preregistered_and_bound():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v2']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v2']
    assert profile_contract == selection_contract
    assert profile_contract['contract_id'] == (
        'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v2')
    assert profile_contract['status'] == 'preregistered_not_executed'
    assert profile_contract['predecessor']['status'] == 'FAIL_CLOSED'
    assert profile_contract['input']['tree_sha256'] == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926')
    execution = profile_contract['execution']
    assert execution['image_tag'] == (
        'm6a10-v2a-fixed10-v2-lidarslam-ours:jazzy')
    assert execution['image_digest'] == (
        'sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
    assert execution['launch']['path'] == (
        'lidarslam/launch/rko_lio_slam.launch.py')
    assert execution['launch']['sha256'] == (
        'd45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c')
    assert execution['launch']['installed_path'] == (
        '/opt/ours_ws/install/lidarslam/share/lidarslam/launch/'
        'rko_lio_slam.launch.py')
    assert execution['launch']['installed_sha256'] == execution['launch'][
        'sha256']
    assert execution['launch']['image_label'] == (
        'benchmark.ours.launch_sha256')
    assert profile_contract['command']['phase_mode'] == 'unpaced_ack'
    assert profile_contract['command']['skip_map_save'] is True
    assert profile_contract['command']['benchmark_no_map_artifacts_env'] == (
        'M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1')
    no_map = profile_contract['no_map_artifact_contract']
    assert no_map['status'] == 'preregistered_not_executed'
    contract_test = no_map['contract_test']
    assert contract_test['path'] == (
        'graph_based_slam/test/test_m6a10_no_map_artifacts.py')
    contract_test_path = ROOT / contract_test['path']
    assert hashlib.sha256(contract_test_path.read_bytes()).hexdigest() == (
        contract_test['sha256'])
    assert contract_test['sha256'] == selection_contract[
        'no_map_artifact_contract']['contract_test']['sha256']
    assert no_map['launch_override'] == {
        'use_save_map_in_loop': False,
        'save_pose_graph_path': '',
    }
    assert no_map['verification'] == (
        'runner_assert_no_map_artifacts_after_launch_termination')
    assert set(no_map['forbidden_relative_paths']) >= {
        'map.pcd', 'pointcloud_map', 'pose_graph.g2o', 'map_bundle.yaml'}
    assert profile_contract['output']['root'].endswith(
        'ours_m6a10_v2a_unpaced_ack_fixed10_v2')
    assert profile_contract['safety']['ground_truth_content_opened'] is False
    assert profile_contract['safety']['scorer_invoked'] is False


def test_m6a10_fixed10_v2_failure_and_v3_quiescence_are_bound():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_root = profile['runtime_policy']['phase_contract_v2']
    selection_root = selection['m6a10_phase_contract']['v2_additive_contract']
    failed = profile_root['ours_m6a10_v2a_unpaced_ack_fixed10_v2'][
        'fixed10_v2_attempt']
    assert failed == selection_root['ours_m6a10_v2a_unpaced_ack_fixed10_v2'][
        'fixed10_v2_attempt']
    assert failed['status'] == 'FAIL_CLOSED'
    assert failed['execution_status'] == 125
    assert failed['retry_count'] == 0
    assert failed['consumer_status'] == 'PASS'
    assert failed['consumer_expected_messages'] == 230895
    assert failed['callback_latency_seconds'] > failed[
        'callback_latency_limit_seconds']
    assert failed['phase_failure_reason'] == 'missing_phase_boundary_input_end'
    assert failed['rss_jitter_percent'] > failed['rss_jitter_limit_percent']
    assert failed['ground_truth_content_opened'] is False
    assert failed['scorer_invoked'] is False

    v3 = profile_root['ours_m6a10_v2a_unpaced_ack_fixed10_v3']
    assert v3 == selection_root['ours_m6a10_v2a_unpaced_ack_fixed10_v3']
    assert v3['status'] == 'preregistered_not_executed'
    assert v3['result'] is None
    assert v3['predecessor']['status'] == 'FAIL_CLOSED'
    assert v3['performance_comparison'] is False
    assert v3['input']['tree_sha256'] == profile_root[
        'ours_m6a10_v2a_unpaced_ack_fixed10_v2']['input']['tree_sha256']
    assert v3['input']['expected_consumer_count'] == 230895
    assert v3['execution']['image_digest'].startswith('sha256:')
    assert v3['command']['max_online_compute_rtf'] == 1.0
    assert v3['no_map_artifact_contract']['require_absent'] is True
    quiescence = v3['quiescence_preflight']
    assert quiescence['status'] == 'preregistered_not_run'
    assert quiescence['receipt_sha256'] is None
    assert quiescence['receipt_sha256_required'] is True
    assert quiescence['runner_start_requires_pass'] is True
    assert quiescence['script']['path'] == 'scripts/check_m6a10_quiescence.py'
    script_path = ROOT / quiescence['script']['path']
    assert hashlib.sha256(script_path.read_bytes()).hexdigest() == (
        quiescence['script']['sha256'])
    test_path = ROOT / quiescence['test']['path']
    assert hashlib.sha256(test_path.read_bytes()).hexdigest() == (
        quiescence['test']['sha256'])
    failed_preflight = v3['quiescence_attempt']
    assert failed_preflight['status'] == 'FAIL_CLOSED'
    assert failed_preflight['receipt_sha256'] == (
        '74732fbdeb9bb8da094bacde3518cc2475eaeb29a3f4fec5c39d7972dc97a5a6')
    assert failed_preflight['runner_start_allowed'] is False
    assert failed_preflight['cpu_busy_percent'] > (
        failed_preflight['max_cpu_busy_percent'])
    assert failed_preflight['load1_per_cpu'] > (
        failed_preflight['max_load1_per_cpu'])
    assert failed_preflight['forbidden_process_count'] == 8
    v4 = profile_root['ours_m6a10_v2a_unpaced_ack_fixed10_v4']
    assert v4 == selection_root['ours_m6a10_v2a_unpaced_ack_fixed10_v4']
    assert v4['status'] == 'preregistered_not_executed'
    assert v4['result'] is None
    assert v4['predecessor'] == {
        'contract_id': 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v3',
        'status': 'PREFLIGHT_FAIL_CLOSED',
        'preflight_receipt_sha256': failed_preflight['receipt_sha256'],
        'retry_count': 0,
    }
    assert v4['quiescence_preflight']['receipt_sha256'] is None
    v4_attempt = v4['quiescence_attempt']
    assert v4_attempt['status'] == 'FAIL_CLOSED'
    assert v4_attempt['receipt_sha256'] == (
        'b8c7b38debd07586f3af060e51b83a5fa94108ba2c80bf27f481ac16f291fb51')
    assert v4_attempt['runner_start_allowed'] is False
    assert v4_attempt['retry_count'] == 0
    assert v4_attempt['cpu_busy_percent'] > v4_attempt['max_cpu_busy_percent']
    assert v4_attempt['load1_per_cpu'] > v4_attempt['max_load1_per_cpu']
    assert v4_attempt['forbidden_process_count'] == 7
    assert v4_attempt['forbidden_process_classes'] == ['compiler']
    assert v4_attempt['ground_truth_content_opened'] is False
    assert v4_attempt['scorer_invoked'] is False
    v5 = profile_root['ours_m6a10_v2a_unpaced_ack_fixed10_v5']
    assert v5 == selection_root['ours_m6a10_v2a_unpaced_ack_fixed10_v5']
    assert v5['contract_id'] == (
        'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v5')
    assert v5['status'] == 'preregistered_not_executed'
    assert v5['result'] is None
    assert v5['predecessor'] == {
        'contract_id': 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v4',
        'status': 'PREFLIGHT_FAIL_CLOSED',
        'preflight_receipt_sha256': v4_attempt['receipt_sha256'],
        'retry_count': 0,
    }
    assert v5['output']['root'].endswith('ours_m6a10_v2a_unpaced_ack_fixed10_v5')
    assert v5['quiescence_preflight']['receipt_sha256'] is None
    assert v5['quiescence_preflight']['receipt_sha256_required'] is True
    assert v5['quiescence_preflight']['runner_start_requires_pass'] is True
    v5_attempt = v5['quiescence_attempt']
    assert v5_attempt['status'] == 'FAIL_CLOSED'
    assert v5_attempt['receipt_sha256'] == (
        'dbf54a1a5a1fd9d527fef280f633e3710b9a79d04ac482bb13ac1278516b4dee')
    assert v5_attempt['runner_start_allowed'] is False
    assert v5_attempt['retry_count'] == 0
    assert v5_attempt['cpu_busy_percent'] > v5_attempt[
        'max_cpu_busy_percent']
    assert v5_attempt['load1_per_cpu'] <= v5_attempt[
        'max_load1_per_cpu']
    assert v5_attempt['forbidden_process_count'] == 1
    assert v5_attempt['forbidden_process_classes'] == ['compiler']
    assert v5_attempt['ground_truth_content_opened'] is False
    assert v5_attempt['scorer_invoked'] is False
    v6 = profile_root['ours_m6a10_v2a_unpaced_ack_fixed10_v6']
    assert v6 == selection_root['ours_m6a10_v2a_unpaced_ack_fixed10_v6']
    assert v6['contract_id'] == (
        'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v6')
    assert v6['status'] == 'FAIL_CLOSED'
    assert v6['result'] is None
    assert v6['predecessor'] == {
        'contract_id': 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v5',
        'status': 'PREFLIGHT_FAIL_CLOSED',
        'preflight_receipt_sha256': v5_attempt['receipt_sha256'],
        'retry_count': 0,
    }
    assert v6['output']['root'].endswith('ours_m6a10_v2a_unpaced_ack_fixed10_v6')
    assert v6['failure_closure'] == {
        'status': 'FAIL_CLOSED',
        'failure_kind': 'runner_not_started_after_preflight',
        'receipt_path': (
            '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
            'ours_m6a10_v2a_unpaced_ack_fixed10_v6/'
            'runner_not_started_after_preflight.json'),
        'receipt_sha256': (
            'ef04937fd7c17b2ed017902985502aa02b9349ccb63961dcd206c0370aea79ad'),
        'runner_start_attempted': False,
        'retry_count': 0,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    assert v6['quiescence_preflight']['status'] == 'PASS'
    assert v6['quiescence_preflight']['receipt_sha256'] == (
        'd9cf50b07157a7da8655d672d7955a768676361735967e35e64367b6272cf3fa')
    assert v6['quiescence_preflight']['runner_start_allowed'] is True
    assert v6['quiescence_preflight']['receipt_sha256_required'] is True
    assert v6['quiescence_preflight']['runner_start_requires_pass'] is True
    assert v6['safety']['ground_truth_content_opened'] is False
    assert v6['safety']['scorer_invoked'] is False
    assert v5['safety']['ground_truth_content_opened'] is False
    assert v5['safety']['scorer_invoked'] is False
    assert v3['safety']['ground_truth_content_opened'] is False
    assert v3['safety']['scorer_invoked'] is False


def test_m6a10_fixed10_v7_identity_failure_is_bound():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v7']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v7']
    assert profile_contract == selection_contract
    assert profile_contract['contract_id'] == (
        'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v7')
    assert profile_contract['status'] == 'FAIL_CLOSED'
    assert profile_contract['result'] is None
    assert profile_contract['performance_comparison'] is False
    assert profile_contract['wall_rtf_role'] == 'diagnostic_only'
    assert profile_contract['failure_closure'] == {
        'status': 'FAIL_CLOSED',
        'failure_kind': 'IDENTITY_PREFLIGHT_FAIL_CLOSED',
        'receipt_path': (
            '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
            'ours_m6a10_v2a_unpaced_ack_fixed10_v7/closure_receipt.json'),
        'receipt_sha256': (
            'fffba96d21721c5b811fb7710d3d77889e81e0c8bfb4ffcb8957f6c94bce57da'),
        'attempt_root_marker_path': (
            '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
            'ours_m6a10_v2a_unpaced_ack_fixed10_v7/attempt_root_marker.json'),
        'attempt_root_marker_sha256': (
            '02aac87246c4e1792ce680c66245e1cefc7cad2fe673dadacebe9e760d77a4ec'),
        'runner_start_attempted': False,
        'quiescence_started': False,
        'retry_count': 0,
        'expected_input_tree_sha256': (
            '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926'),
        'observed_input_tree_sha256': (
            'bcbb4c86f568125104565fca3882695fab0b501a7ca9d9aa9aaa643f1b8ee6eb'),
        'observed_tree_hash_kind': 'launcher_relative_path_content_sha256_v1',
        'canonical_profile_input_tree_sha256': (
            '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926'),
        'canonical_profile_tree_hash_kind': 'relative_path_size_content_sha256_v1',
        'canonical_profile_tree_hash_matches_expected': True,
        'diagnosis': 'launcher_hash_omitted_file_size_field',
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    assert profile_contract['predecessor'] == {
        'contract_id': 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v6',
        'status': 'FAIL_CLOSED',
        'failure_kind': 'runner_not_started_after_preflight',
        'closure_receipt_sha256': (
            'ef04937fd7c17b2ed017902985502aa02b9349ccb63961dcd206c0370aea79ad'),
        'retry_count': 0,
    }
    launcher = profile_contract['launcher']
    launcher_path = ROOT / launcher['path']
    test_path = ROOT / launcher['test_path']
    assert hashlib.sha256(launcher_path.read_bytes()).hexdigest() == launcher['sha256']
    assert hashlib.sha256(test_path.read_bytes()).hexdigest() == launcher['test_sha256']
    phase_contract = profile_contract['execution']['phase_contract']
    phase_path = ROOT / phase_contract['path']
    assert hashlib.sha256(phase_path.read_bytes()).hexdigest() == phase_contract['sha256']
    assert launcher['quiescence_invocations'] == 1
    assert launcher['runner_start_same_process_tree'] is True
    assert launcher['immediate_start_after_pass'] is True
    assert launcher['shell_interpolation'] is False
    assert launcher['taskset_or_cpuset'] == 'forbidden'
    assert launcher['preflight_to_runner_max_gap_seconds'] == 2.0
    assert launcher['host_time_argv'] == ['/usr/bin/time', '-v', '-o', 'time-v.txt']
    assert profile_contract['input']['expected_consumer_count'] == 230895
    assert profile_contract['quiescence_preflight']['receipt_sha256'] is None
    assert profile_contract['quiescence_preflight']['runner_start_requires_pass'] is True
    assert profile_contract['safety']['ground_truth_content_opened'] is False
    assert profile_contract['safety']['scorer_invoked'] is False


def test_m6a10_fixed10_v8_identity_failure_and_tree_hash_are_bound():
    profile = _profile()
    selection = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    profile_contract = profile['runtime_policy']['phase_contract_v2'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v8']
    selection_contract = selection['m6a10_phase_contract']['v2_additive_contract'][
        'ours_m6a10_v2a_unpaced_ack_fixed10_v8']
    assert profile_contract == selection_contract
    assert profile_contract['status'] == 'FAIL_CLOSED'
    assert profile_contract['result'] is None
    assert profile_contract['predecessor']['status'] == 'FAIL_CLOSED'
    assert profile_contract['predecessor']['closure_receipt_sha256'] == (
        'fffba96d21721c5b811fb7710d3d77889e81e0c8bfb4ffcb8957f6c94bce57da')
    assert profile_contract['input']['tree_sha256'] == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
    assert len(profile_contract['input']['tree_sha256']) == 64
    assert profile_contract['input']['tree_hash_kind'] == (
        'relative_path_size_content_sha256_v1')
    launcher = profile_contract['launcher']
    launcher_path = ROOT / launcher['path']
    test_path = ROOT / launcher['test_path']
    assert hashlib.sha256(launcher_path.read_bytes()).hexdigest() == launcher['sha256']
    assert hashlib.sha256(test_path.read_bytes()).hexdigest() == launcher['test_sha256']
    assert launcher['pinned_v7_launcher_sha256'] == (
        '47ab6b059bc9736da5fa69933d7a1d1024db186b78b1c5d30b987544fb0d20d4')
    assert launcher['tree_hash_kind'] == 'relative_path_size_content_sha256_v1'
    helper_path = ROOT / launcher['tree_hash_helper_path']
    assert hashlib.sha256(helper_path.read_bytes()).hexdigest() == (
        launcher['tree_hash_helper_sha256'])
    assert launcher['preflight_to_runner_max_gap_seconds'] == 2.0
    assert profile_contract['output']['root'].endswith(
        'ours_m6a10_v2a_unpaced_ack_fixed10_v8')
    closure = profile_contract['failure_closure']
    assert closure['failure_kind'] == 'IDENTITY_PREFLIGHT_FAIL_CLOSED'
    assert closure['runner_start_attempted'] is False
    assert closure['quiescence_started'] is False
    assert closure['retry_count'] == 0
    assert closure['observed_local_image_id'] == (
        'sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
    assert closure['profile_image_digest'] == closure['observed_local_image_id']
    assert profile_contract['quiescence_preflight']['receipt_sha256'] is None
    assert profile_contract['safety']['ground_truth_content_opened'] is False
    assert profile_contract['safety']['scorer_invoked'] is False


def test_m6a5_memory_contract_and_campaign_lineage_are_bound():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    contract = receipt['m6a5_memory_contract']
    assert contract['measurement_version'] == 'm6a5-cgroup-v2-memory-v1'
    assert contract['measurement_scope'] == 'container_cgroup_v2'
    assert contract['comparative_rss_field'] == 'container_cgroup_peak_bytes'
    assert contract['docker_client_comparable'] is False
    assert contract['known_allocation_peak_delta_bytes'] > 100 * 1024 * 1024
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a5_memory_contract_and_lineage']['pass'] is False
    assert result['checks']['rival_source_closure_identity']['pass'] is False
    assert result['status'] == 'INVALID'


def test_m6a5_memory_contract_tamper_is_fail_closed():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['m6a5_memory_contract']['docker_client_comparable'] = True
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a5_memory_contract_and_lineage']['pass'] is False
    assert any('docker_client_comparable' in item for item in result['errors'])


def test_m6a7_process_rss_contract_and_audit_are_bound():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    contract = receipt['m6a7_process_rss_contract']
    assert contract['status'] == 'PASS'
    assert contract['primary_metric'] == 'aggregate_process_tree_peak_rss_bytes'
    assert contract['primary_metric_definition'] == (
        'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
    assert contract['memory_max'] == 'max'
    assert contract['docker_client_comparable'] is False
    assert contract['schedule'] == {
        'order': 'AB_BA_alternating', 'pairs': 20, 'runs': 40,
        'all_complete': True}
    assert contract['blind_scope'] == {
        'ground_truth_content_opened': False, 'scorer_invoked': False,
        'campaign4_started': False}
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a7_process_rss_contract']['pass'] is False
    assert result['checks']['rival_source_closure_identity']['pass'] is False
    assert result['status'] == 'INVALID'


def test_m6a7_process_rss_contract_tamper_is_fail_closed():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['m6a7_process_rss_contract']['memory_max'] = '4g'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['m6a7_process_rss_contract']['pass'] is False
    assert any('memory_max' in item for item in result['errors'])


def test_observed_identity_is_complete_after_external_freeze():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['machine_fingerprint']['pass'] is False
    assert result['checks']['thread_policy_complete']['pass'] is True
    assert result['checks']['system_ours']['evidence']['revision_status'] == 'pinned'
    assert result['checks']['rival_source_closure_identity']['pass'] is False
    assert result['status'] == 'INVALID'
    assert result['pass'] is False


def test_external_container_config_is_bound_to_immutable_image():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    config = receipt['systems']['fast_livo2']['configs'][1]
    assert config['path_kind'] == 'external_container_absolute_path'
    assert config['hash_kind'] == 'external_container_file_sha256'
    assert config['status'] == 'observed'
    assert config['container_image_digest'] == receipt['systems']['fast_livo2'][
        'container']['image_digest']
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_fast_livo2']['pass'] is False
    assert result['checks']['rival_source_closure_identity']['pass'] is False

    tampered = copy.deepcopy(receipt)
    tampered['systems']['fast_livo2']['configs'][1][
        'container_image_digest'] = 'sha256:' + '0' * 64
    result = _CHECKER.evaluate(tampered, profile_document)
    assert result['checks']['system_fast_livo2']['pass'] is False
    assert any('container_image_digest does not match image' in item
               for item in result['errors'])


def test_canonical_profile_hash_excludes_only_registered_receipt_sha():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    original = copy.deepcopy(document)
    expected = _CHECKER.canonical_profile_sha256(document)
    assert document == original
    document['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_sha256'] = '0' * 64
    assert _CHECKER.canonical_profile_sha256(document) == expected


def test_canonical_profile_hash_changes_for_non_receipt_mutations():
    document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    expected = _CHECKER.canonical_profile_sha256(document)
    mutations = []
    renamed = copy.deepcopy(document)
    renamed['competitive_slam_profile']['name'] += '-mutated'
    mutations.append(renamed)
    policy_changed = copy.deepcopy(document)
    policy_changed['competitive_slam_profile']['evidence_gate_v2'][
        'repetitions'] += 1
    mutations.append(policy_changed)
    dataset_changed = copy.deepcopy(document)
    dataset_changed['competitive_slam_profile']['datasets']['holdout_slots'][
        'holdout_1']['sequence'] += '-mutated'
    mutations.append(dataset_changed)
    assert all(_CHECKER.canonical_profile_sha256(item) != expected
               for item in mutations)
    assert _CHECKER.PROFILE_CANONICAL_HASH_KIND == 'canonical_profile_sha256_v1'


def test_execution_preflight_pending_observation_remains_incomplete():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['status'] = 'pending'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['status'] == 'INVALID'
    assert result['pass'] is False
    assert result['checks']['receipt_path_and_sha256']['pass'] is True
    assert result['checks']['receipt_status_ready']['pass'] is False


def test_execution_preflight_rejects_receipt_hash_mismatch():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    mutated = copy.deepcopy(profile_document)
    mutated['competitive_slam_profile']['evidence_gate_v2'][
        'execution_selection_receipt_sha256'] = '0' * 64
    result = _CHECKER.evaluate(receipt, mutated)
    assert result['status'] == 'INVALID'
    assert result['pass'] is False
    assert any('receipt SHA' in item for item in result['errors'])


def _mark_system_ready(receipt, system):
    item = receipt['systems'][system]
    item['repository']['revision_status'] = 'ready'
    item['repository']['worktree_dirty'] = False
    for key in ('tracked_diff_sha256', 'untracked_content_sha256',
                'clean_provenance_sha256'):
        item['repository'][key] = 'a' * 64
    item['container']['status'] = 'ready'
    item['container']['image_digest'] = 'sha256:' + 'b' * 64
    item['toolchain']['status'] = 'ready'
    item['toolchain']['fingerprint'] = 'c' * 64


def test_execution_preflight_ready_status_but_dirty_ours_is_not_ready():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['status'] = 'ready'
    _mark_system_ready(receipt, 'ours')
    receipt['systems']['ours']['repository']['worktree_dirty'] = True
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_ours']['pass'] is False
    assert any('worktree must be clean' in item for item in result['errors'])


def test_execution_preflight_pending_container_and_toolchain_cannot_pass():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    _mark_system_ready(receipt, 'glim')
    receipt['systems']['glim']['container']['status'] = 'pending_build'
    receipt['systems']['glim']['toolchain']['status'] = 'pending_build'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_glim']['pass'] is False
    assert any('glim.container.status' in item for item in result['errors'])
    assert any('glim.toolchain.status' in item for item in result['errors'])


def test_execution_preflight_system_diagnostics_are_independent():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    _mark_system_ready(receipt, 'glim')
    receipt['systems']['glim']['repository']['worktree_dirty'] = False
    # The checked-in receipt now has real ours observations; make this system
    # explicitly unresolved so the per-system diagnostic remains meaningful.
    receipt['systems']['ours']['container']['status'] = 'pending_build'
    receipt['systems']['ours']['toolchain']['status'] = 'pending_build'
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['checks']['system_glim']['pass'] is False
    assert result['checks']['system_ours']['pass'] is False
    assert result['checks']['all_systems_pinned_and_resolved']['evidence']['per_system'][
        'glim'] is False
    assert result['checks']['rival_source_closure_identity']['pass'] is False


def test_execution_preflight_composite_scorer_mismatch_is_invalid():
    profile_document = yaml.safe_load(PROFILE_PATH.read_text(encoding='utf-8'))
    receipt = yaml.safe_load(EXECUTION_RECEIPT_PATH.read_text(encoding='utf-8'))
    receipt['common_identity']['scorer']['canonical_fingerprint'] = '0' * 64
    result = _CHECKER.evaluate(receipt, profile_document)
    assert result['status'] == 'INVALID'
    assert result['checks']['scorer_files_and_fingerprint']['pass'] is False
    assert result['checks']['rival_source_closure_identity']['pass'] is False
    assert any('rival_source_closure' in item
               for item in result['errors'])


def test_execution_preflight_cli_emits_ready_json_yaml_identity(tmp_path):
    json_path = tmp_path / 'preflight.json'
    yaml_path = tmp_path / 'preflight.yaml'
    environment = os.environ.copy()
    environment.pop('PYTHONPATH', None)
    environment['PYTHONDONTWRITEBYTECODE'] = '1'
    completed = subprocess.run([
        'python3', str(EXECUTION_CHECKER_PATH),
        '--receipt', str(EXECUTION_RECEIPT_PATH),
        '--profile', str(PROFILE_PATH),
        '--output', str(json_path),
        '--yaml-output', str(yaml_path),
    ], check=False, capture_output=True, text=True, env=environment)
    # The retained r1 receipt is intentionally rejected after the r2 closure
    # became current; this CLI must not emit a false READY/PASS result.
    assert completed.returncode != 0
    json_result = json.loads(json_path.read_text(encoding='utf-8'))
    yaml_result = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert json_result['status'] == 'INVALID'
    assert yaml_result['status'] == json_result['status']
    assert json_result['pass'] is False
    assert json_result['checks']['rival_source_closure_identity']['pass'] is False
    assert json_result['checks']['dataset_source_closure']['pass'] is False
    assert json_result['dataset_source_closure']['status'] == 'NOT_READY'
    for key in ('profile_sha256', 'execution_receipt_sha256',
                'canonical_scorer_fingerprint',
                'thread_policy_canonical_sha256'):
        assert key in json_result['identity']


def test_comparison_modalities_are_not_mixed_between_rivals():
    tracks = _profile()['tracks']
    assert tracks['glim_cpu_lidar_imu']['required_modalities'] == ['lidar', 'imu']
    assert tracks['fast_livo2_lidar_imu_visual']['required_modalities'] == [
        'lidar', 'imu', 'monocular_camera'
    ]


def test_online_phase_is_primary_and_wall_rtf_is_diagnostic():
    runtime = _profile()['runtime_policy']
    assert runtime['gate_metric'] == 'runtime.online_compute_rtf'
    assert runtime['maximum_online_compute_rtf'] == 1.0
    assert runtime['phase_contract_version'] == 'm6a10-online-compute-v1'
    assert runtime['exclude_map_save_postprocess'] is True
    assert runtime['exclude_fixed_shutdown_grace'] is True
    assert runtime['legacy_wall_gate_metric'] == (
        'runtime.processing_realtime_factor')
    assert runtime['legacy_wall_gate_role'] == 'diagnostic_only'
    assert runtime['replay_wall_realtime_factor_is_diagnostic_only'] is True
    assert runtime['maximum_trajectory_end_gap_seconds'] <= 0.25
    aggregation = _profile()['repetition_aggregation']
    assert aggregation['processing_realtime_factor'] == 'median'
    assert aggregation['peak_rss'] == 'maximum'
    assert aggregation['completion_and_failures'] == 'worst_case'
    assert aggregation['mapping_quality'] == 'worst_case'


def test_cross_ros_comparison_requires_semantic_sensor_identity():
    execution = _profile()['execution_contract']
    assert execution['same_sensor_messages'] is True
    assert execution['cross_ros_representation_policy'] == (
        'canonical_deserialized_message_digest')
    assert execution['require_cross_ros_topic_counts_equal'] is True
    assert execution['require_cross_ros_record_timestamps_equal'] is True
    assert execution['require_cross_ros_all_sensor_fields_equal'] is True
    assert execution['ignored_cross_ros_transport_fields'] == [
        'std_msgs/Header.seq']


def test_competition_rko_config_explicitly_disables_relocalization_scope():
    config = yaml.safe_load(HILTI_RKO_PATH.read_text(encoding='utf-8'))
    assert config['enable_kidnap_relocalization'] is False
    assert config['reset_on_registration_failure'] is False
    assert config['relocalize_after_scan_gap'] is False
