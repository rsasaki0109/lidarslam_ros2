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

"""Static contracts for the FAST-LIVO2 M6a10-v2c implementation."""

import argparse
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
WRAPPER = ROOT / 'scripts/fast_livo2_container_run.sh'
RUNNER_PATH = ROOT / 'scripts/run_fast_livo2_benchmark.py'
PATCH = ROOT / 'docker/patches/fast_livo2.m6a10-v2c.patch'
PREFLIGHT = ROOT / 'scripts/preflight_fast_livo2_m6a10_v2c.py'


def _sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _runner():
    spec = importlib.util.spec_from_file_location('fast_runner', RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _contract():
    return yaml.safe_load(PROFILE.read_text())['competitive_slam_profile'][
        'm6a10_fast_livo2_v2c']


def test_v2c_profile_retry_status_and_counts_are_exact():
    contract = _contract()
    assert contract['status'] == 'FAIL_CLOSED'
    assert contract['result'] is None
    prior = contract['prior_failure_closure']
    assert prior['failure_kind'] == 'quiescence_fail_closed'
    assert prior['runner_start_attempted'] is False
    assert prior['retry_count'] == 0
    assert prior['receipt_sha256'] == (
        'b3dc7ed143452e605f6315361fa7f3a909051bb05894b5e20c27737ffb754476')
    retry = contract['retry_v2']
    assert retry['status'] == 'FAIL_CLOSED'
    assert retry['failure_kind'] == 'runner_contract_validation'
    assert retry['retry_count'] == 0
    assert retry['runner_start_attempted'] is True
    assert retry['container_start_attempted'] is False
    assert retry['bag_replay_started'] is False
    assert retry['runner_exit_status'] == 1
    assert retry['error'] == 'missing_safety_ground_truth_mount_exposed'
    closure = Path(retry['receipt_path'])
    assert closure.exists()
    assert _sha256(closure) == retry['receipt_sha256']
    closure_doc = json.loads(closure.read_text())
    assert closure_doc['status'] == 'FAIL_CLOSED'
    assert closure_doc['container_start_attempted'] is False
    assert closure_doc['bag_replay_started'] is False
    assert contract['quiescence_attempt']['status'] == 'PASS'
    assert contract['quiescence_attempt']['runner_start_allowed'] is True
    counts = contract['consumer']['expected_topic_counts']
    assert counts == {'lidar': 5793, 'imu': 225102, 'image': 5792}
    assert contract['consumer']['expected_messages'] == sum(counts.values())
    assert contract['input']['expected_messages'] == sum(counts.values())
    assert contract['input']['canonical_ros2_tree_sha256'] == (
        '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
    assert contract['safety'] == {
        'ground_truth_content_opened': False,
        'ground_truth_mount_exposed': False,
        'scorer_invoked': False,
        'benchmark_input_only': True,
        'retry': 0,
    }
    retry_v3 = contract['retry_v3']
    assert retry_v3['status'] == 'FAIL_CLOSED'
    assert retry_v3['failure_kind'] == 'quiescence_fail_closed'
    assert retry_v3['receipt_sha256'] == (
        'eed5aa5072048ea77007a2f38a1440ca60af743d389a561b9aa89b5503bbc399')
    assert retry_v3['quiescence_receipt_sha256'] == (
        'd27cefb4923848eed622829a9c802adb378f4e2d2f1078884593e246a8702cf7')
    assert retry_v3['retry_count'] == 0
    assert retry_v3['predecessor_receipt_sha256'] == retry['receipt_sha256']
    assert retry_v3['runner_start_attempted'] is False
    assert retry_v3['container_start_attempted'] is False
    assert retry_v3['bag_replay_started'] is False
    assert retry_v3['result'] is None
    retry_v3_closure = Path(retry_v3['receipt_path'])
    assert _sha256(retry_v3_closure) == retry_v3['receipt_sha256']
    assert json.loads(retry_v3_closure.read_text())['status'] == 'FAIL_CLOSED'
    retry_v4 = contract['retry_v4']
    assert retry_v4['status'] == 'FAIL_CLOSED'
    assert retry_v4['failure_kind'] == 'no_progress_evidence_under_bounded_supervision'
    assert retry_v4['retry_count'] == 0
    assert retry_v4['predecessor_receipt_sha256'] == retry_v3['receipt_sha256']
    assert retry_v4['runner_start_attempted'] is True
    assert retry_v4['container_start_attempted'] is True
    assert retry_v4['bag_replay_started'] is True
    assert retry_v4['runner_exit_status'] == 2
    assert retry_v4['container_exit_status'] == 137
    assert retry_v4['first_consumer_ack_observed'] is False
    assert retry_v4['consumer_evidence_present'] is False
    closure_v4 = Path(retry_v4['receipt_path'])
    assert closure_v4.exists()
    assert _sha256(closure_v4) == retry_v4['receipt_sha256']
    assert json.loads(closure_v4.read_text())['status'] == 'FAIL_CLOSED'
    correction_v4 = Path(retry_v4['diagnostic_correction_path'])
    assert correction_v4.exists()
    assert _sha256(correction_v4) == retry_v4['diagnostic_correction_sha256']
    assert json.loads(correction_v4.read_text())[
        'attempt']['first_ack_absence_proven'] is False
    assert contract['quiescence_attempt_v4']['status'] == 'PASS'
    assert contract['quiescence_attempt_v4']['runner_start_allowed'] is True
    assert contract['output']['root'] == retry_v4['output_root']


@pytest.mark.parametrize('mount_exposed', [None, True])
def test_gt_mount_safety_missing_or_true_fails_before_container(
        monkeypatch, tmp_path, mount_exposed):
    runner = _runner()
    phase = json.loads(json.dumps(_contract()))
    bag = tmp_path / 'synthetic-input.bag'
    bag.write_bytes(b'not-replayed')
    phase['input']['path'] = str(bag)
    phase['input']['bytes'] = bag.stat().st_size
    phase['input']['sha256'] = _sha256(bag)
    if mount_exposed is None:
        phase['safety'].pop('ground_truth_mount_exposed')
    else:
        phase['safety']['ground_truth_mount_exposed'] = mount_exposed
    monkeypatch.setattr(
        runner, 'load_m6a10_fast_contract', lambda _profile: phase)
    monkeypatch.setattr(
        runner, 'validate_m6a10_image_identity',
        lambda _phase: pytest.fail('container identity must not be inspected'))
    monkeypatch.setattr(
        runner.subprocess, 'run',
        lambda *args, **kwargs: pytest.fail('container must not start'))
    args = argparse.Namespace(
        profile=PROFILE, bag=bag, output=tmp_path / 'output',
        phase_mode='unpaced_ack')
    with pytest.raises((KeyError, ValueError)):
        runner.run_m6a10_phase(args)
    assert not args.output.exists()


def test_selection_mirrors_v2c_retry_identity_without_a_result():
    profile = _contract()
    selection = yaml.safe_load(SELECTION.read_text())['m6a10_fast_livo2_v2c']
    assert selection['contract_id'] == profile['contract_id']
    assert selection['status'] == 'FAIL_CLOSED'
    assert selection['result'] is None
    assert selection['prior_failure_closure'] == profile['prior_failure_closure']
    assert selection['retry_v2'] == profile['retry_v2']
    assert selection['retry_v3'] == profile['retry_v3']
    assert selection['retry_v4'] == profile['retry_v4']
    assert selection['quiescence_attempt'] == profile['quiescence_attempt']
    assert selection['quiescence_attempt_v4'] == profile['quiescence_attempt_v4']
    assert selection['quiescence_attempt_v5'] == profile['quiescence_attempt_v5']
    assert selection['input']['sha256'] == profile['input']['sha256']
    assert selection['input']['expected_messages'] == 236687
    assert selection['feeder']['sha256'] == profile['feeder']['sha256']
    assert selection['runner']['wrapper_sha256'] == (
        'f4236ec79659becacb12dc5f76c2d7ef41bc83be7616df88c1ee6ebc60e65d20')


def test_v2c_hash_bindings_match_current_files():
    profile = _contract()
    selection = yaml.safe_load(SELECTION.read_text())['m6a10_fast_livo2_v2c']
    assert profile['source']['patch_sha256'] == _sha256(PATCH)
    assert profile['feeder']['sha256'] == _sha256(FEEDER)
    assert profile['runner']['sha256'] == _sha256(RUNNER_PATH)
    assert profile['runner']['phase_entrypoint_sha256'] == _sha256(WRAPPER)
    assert profile['execution']['recipe']['sha256'] == _sha256(
        ROOT / 'docker/fast_livo2_benchmark.Dockerfile')
    assert profile['execution']['build_entrypoint']['sha256'] == _sha256(
        ROOT / 'scripts/build_competitive_benchmark_images.sh')
    assert selection['patch']['sha256'] == profile['source']['patch_sha256']
    assert selection['runner']['sha256'] == profile['runner']['sha256']


def test_execution_identity_preflight_is_passed_and_input_blind():
    profile = _contract()
    execution = profile['execution']
    receipt_path = Path(execution['execution_identity_preflight_path'])
    assert execution['execution_identity_preflight_status'] == 'PASS'
    assert hashlib.sha256(receipt_path.read_bytes()).hexdigest() == (
        execution['execution_identity_preflight_sha256'])
    assert execution['execution_identity_preflight_script_sha256'] == (
        _sha256(PREFLIGHT))
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    assert receipt['receipt_kind'] == (
        'fast_livo2_m6a10_v2c_execution_identity_preflight')
    assert receipt['status'] == 'PASS'
    assert receipt['identity']['image_id'] == (
        'sha256:89f6baeca8bc3691ed8465d0e19fd962cce708053cd28c8a4eebabb5b841f0ea')
    assert receipt['identity']['preflight_script']['sha256'] == (
        _sha256(PREFLIGHT))
    mount_plan = receipt['execution_mount_plan']
    assert mount_plan['network'] == 'none'
    assert mount_plan['rootfs'] == 'read_only'
    assert mount_plan['input_mount_performed'] is False
    assert mount_plan['observed_probe_mounts'] == []
    assert mount_plan['ground_truth_mount_forbidden'] is True
    assert mount_plan['scorer_mount_forbidden'] is True
    assert receipt['input_identity_expected_only'][
        'content_read_during_preflight'] is False
    assert receipt['safety'] == {
        'bag_replay_started': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'retry': 0,
    }


def test_feeder_proves_one_inflight_ack_and_never_reads_gt():
    text = FEEDER.read_text()
    assert 'publish_one_inflight' in text
    assert 'publisher.publish(message)' in text
    assert 'wait_for_callback' in text
    assert 'ack_proxy()' in text
    assert repr('single_inflight') + ': True' in text
    assert repr('ground_truth_content_opened') + ': False' in text
    assert repr('scorer_invoked') + ': False' in text
    assert 'rosbag.play' not in text
    assert '/bench' not in text
    assert 'ground_truth_path' not in text.lower()
    assert 'gt_mount' not in text.lower()


def test_v2c_command_mounts_only_raw_bag_and_requires_verified_image(
        tmp_path):
    runner = _runner()
    contract = _contract()
    args = argparse.Namespace(phase_mode='unpaced_ack')
    command = runner.build_m6a10_fast_command(
        args, contract, tmp_path, Path(contract['input']['path']))
    rendered = ' '.join(command)
    assert '--network none' in rendered
    assert f"{contract['input']['path']}:/input/raw_input.bag:ro" in rendered
    assert ':/bench' not in rendered
    assert 'ground_truth' not in rendered.lower()
    assert 'scorer' not in rendered.lower()
    assert '/runner:ro' in rendered
    assert contract['execution']['image_digest'] == (
        'sha256:89f6baeca8bc3691ed8465d0e19fd962cce708053cd28c8a4eebabb5b841f0ea')
    assert contract['execution']['image_status'] == 'build_passed_not_executed'
    immutable = 'sha256:' + 'a' * 64
    contract['execution']['image_digest'] = immutable
    immutable_command = runner.build_m6a10_fast_command(
        args, contract, tmp_path, Path(contract['input']['path']))
    assert immutable_command[-2] == immutable


def test_patch_contains_status_ack_services_and_no_map_guard():
    text = PATCH.read_text()
    for marker in (
            'consumer_status', 'consumer_ack', 'feeder_acknowledged_',
            'm6a10_queue_size',
            'if (!m6a10_consumer_evidence.enabled()) savePCD()'):
        assert marker in text
    assert 'M6A10_FAST_ACK_BACKPRESSURE_VERIFIED") == "1"' not in text


def test_wrapper_v2_never_falls_back_to_rosbag_play_or_map_save():
    text = WRAPPER.read_text()
    v2_anchor = text.index('python3 /runner/scripts/fast_livo2_m6a10_feeder.py')
    v2_start = text.rindex(
        'if [[ "${M6A10_FAST_V2}" == 1 ]]; then', 0, v2_anchor)
    v2_end = text.index('\nelse\n', v2_start)
    v2 = text[v2_start:v2_end]
    assert 'fast_livo2_m6a10_feeder.py' in v2
    assert 'consumer_eof' in v2
    assert 'consumer_finalize' not in v2
    assert '\n    rosbag play' not in v2
    assert 'SAVE_MAP=0' in text
    assert 'feeder_receipt.json' in v2
    assert 'drain_receipt.json' in v2


def test_docker_recipe_pins_v2c_patch_and_feeder_labels():
    text = (ROOT / 'docker/fast_livo2_benchmark.Dockerfile').read_text()
    # ``--inaccurate-eof`` can concatenate an otherwise valid context line
    # when applying this patch, producing an uncompilable LIVMapper header.
    # The recipe must use ordinary git-apply semantics for the pinned patch.
    assert '--inaccurate-eof' not in text
    assert (
        'FAST_LIVO2_M6A10_PATCH_SHA256='
        '33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297'
        in text)
    assert (
        'FAST_LIVO2_M6A10_FEEDER_SHA256='
        'bde0631d29dbb18575fe0fd2ce4bc339e738d14e1b47a1ccc8a77aa348f5622d'
        in text)
    assert 'benchmark.fast_livo2.m6a10_consumer_contract' in text
    assert 'single_inflight_exact_count_observed' in text
