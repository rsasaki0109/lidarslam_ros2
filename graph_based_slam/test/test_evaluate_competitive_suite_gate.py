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


"""Tests for the three-holdout, two-track suite gate."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'evaluate_competitive_suite_gate.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SPEC = importlib.util.spec_from_file_location('suite_gate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
CONTRACT = yaml.safe_load(PROFILE.read_text())['competitive_slam_profile']
FROZEN_CONTRACT = copy.deepcopy(CONTRACT)
for _slot in FROZEN_CONTRACT['datasets']['holdout_slots'].values():
    _slot['status'] = 'frozen'


def _gates(passed=True):
    gates = []
    for slot in CONTRACT['datasets']['holdout_slots'].values():
        for track in MODULE.REQUIRED_TRACKS:
            gates.append({'sequence': slot['sequence'], 'track': track,
                          'pass': passed})
    return gates


def test_every_holdout_must_pass_both_tracks():
    result = MODULE.evaluate(_gates(), FROZEN_CONTRACT)
    assert result['pass'] is True
    assert len(result['complete_holdout_wins']) == 3
    assert result['expected_gate_count'] == 6


def test_missing_or_failed_track_prevents_suite_claim():
    gates = _gates()
    gates.pop()
    missing = MODULE.evaluate(gates, FROZEN_CONTRACT)
    assert missing['pass'] is False
    assert missing['missing_gates']

    gates = _gates()
    gates[0]['pass'] = False
    failed = MODULE.evaluate(gates, FROZEN_CONTRACT)
    assert failed['pass'] is False
    assert failed['failed_gates']


def test_pending_input_hashes_prevent_suite_claim():
    pending = copy.deepcopy(CONTRACT)
    pending['datasets']['holdout_slots']['holdout_1']['status'] = (
        'assigned_inputs_pending_hash')
    result = MODULE.evaluate(_gates(), pending)
    assert result['checks']['all_holdout_inputs_frozen'] is False
    assert result['pass'] is False


def _provenance(system=None, contract=CONTRACT):
    pinned_revision = (contract.get('rivals', {}).get(system, {}).get('revision')
                       if system else None)
    closure_identity = MODULE.current_rival_source_closure_identity(
        {'competitive_slam_profile': contract}, root=ROOT)
    return {
        'input_sha256': 'a' * 64,
        'reference_sha256': 'b' * 64,
        'calibration_sha256': 'c' * 64,
        'machine_id': 'm5-fixture-machine',
        'hardware_fingerprint': '1' * 64,
        'thread_policy': {
            'cpu_affinity': [0, 1],
            'max_threads': 2,
            'omp_num_threads': 1,
            'openblas_num_threads': 1,
            'mkl_num_threads': 1,
            'tbb_num_threads': 1,
            'accelerator_policy': 'cpu',
        },
        'release': 'Release',
        'revision': pinned_revision or 'd' * 40,
        'container_digest': 'sha256:' + 'e' * 64,
        'toolchain_fingerprint': '2' * 64,
        'config_sha256': 'f' * 64,
        'scorer_fingerprint': '3' * 64,
        'rival_source_closure': closure_identity,
    }


def _synthetic_contract():
    contract = copy.deepcopy(CONTRACT)
    ready_receipt = ROOT / 'graph_based_slam' / 'test' / 'fixtures' / (
        'competitive_execution_selection_ready.yaml')
    execution_policy = contract['evidence_gate_v2']
    # The fixture is immutable test input, not an installed production
    # resource.  Bind its absolute identity explicitly so the installed
    # package_root() gate cannot reinterpret this source-only receipt below
    # install/share/graph_based_slam or silently fall back to the checkout.
    execution_policy['execution_selection_receipt_path'] = str(
        ready_receipt.resolve())
    execution_policy['execution_selection_receipt_sha256'] = hashlib.sha256(
        ready_receipt.read_bytes()).hexdigest()
    slots = {}
    for index, dataset in enumerate(('fresh_exp01', 'fresh_exp02', 'fresh_exp03'), 1):
        selection_hash = ('1', '2', '3')[index - 1]
        input_hash = ('4', '5', '6')[index - 1]
        ground_truth_hash = ('7', '8', '9')[index - 1]
        calibration_hash = ('a', 'b', 'c')[index - 1]
        slots[f'fresh_{index}'] = {
            'status': 'frozen_unopened',
            'sequence': dataset,
            'selection_receipt_sha256': selection_hash * 64,
            'input_manifest_sha256': input_hash * 64,
            'ground_truth_sha256': ground_truth_hash * 64,
            'calibration_archive_sha256': calibration_hash * 64,
        }
    contract['datasets']['fresh_holdout_slots'] = slots
    return contract


SYNTHETIC_CONTRACT = _synthetic_contract()


def _v2_evidence(contract=SYNTHETIC_CONTRACT, ours_ape=0.8,
                 glim_ape=1.0, fast_ape=1.1):
    fresh_slots = contract['datasets']['fresh_holdout_slots']
    historical_slots = contract['datasets']['holdout_slots']
    partition_slots = {'historical': historical_slots, 'fresh': fresh_slots}
    datasets = [slot['sequence'] for slots in partition_slots.values()
                for slot in slots.values()]
    systems = {}
    for system, ape in (('ours', ours_ape), ('glim', glim_ape),
                        ('fast_livo2', fast_ape)):
        runs = []
        for partition, slots in partition_slots.items():
            for dataset in [slot['sequence'] for slot in slots.values()]:
                slot = next(slot for slot in slots.values()
                            if slot['sequence'] == dataset)
                for run_index in (1, 2, 3):
                    dataset_identity = {
                        'input_manifest_sha256': slot['input_manifest_sha256'],
                        'reference_sha256': slot['ground_truth_sha256'],
                        'calibration_sha256': slot['calibration_archive_sha256'],
                    }
                    if partition == 'fresh':
                        dataset_identity['selection_receipt_sha256'] = (
                            slot['selection_receipt_sha256'])
                    runs.append({
                        'dataset': dataset,
                        'run_index': run_index,
                        'dataset_identity': dataset_identity,
                        'complete': True,
                        'process_exit_status': 0,
                        'trajectory_complete': True,
                        'sequence_failure': False,
                        'catastrophic_failure': False,
                        'verified_false_loops': 0,
                        'trajectory': {'ape_rmse_m': ape},
                        'runtime': {'processing_rtf': 0.8,
                                    'online_compute_rtf': 0.8,
                                    'peak_rss_mb': 100.0 if system == 'ours' else 110.0},
                        'map': {'plane_thickness_mean_m': 0.04,
                                'plane_thickness_p95_m': 0.08,
                                'planar_coverage': 0.70},
                        'artifacts': {'trajectory_sha256':
                                      ('1' if system == 'ours' else '2') * 64,
                                      'map_sha256':
                                      ('3' if dataset.endswith('02') else '4') * 64},
                        # Synthetic sealed attempt identity.  The real claim
                        # path requires this to be backed by the per-run
                        # execution artifact; deterministic metric bytes may
                        # still be shared by repeated runs.
                        'execution_receipt_sha256': hashlib.sha256(
                            f'{system}:{dataset}:{run_index}:execution'.encode()
                        ).hexdigest(),
                        'execution_receipt_file_sha256': hashlib.sha256(
                            f'{system}:{dataset}:{run_index}:execution-file'.encode()
                        ).hexdigest(),
                        'campaign_id': hashlib.sha256(
                            b'synthetic-competitive-campaign').hexdigest(),
                    })
        systems[system] = {
            'provenance': _provenance(system, contract), 'runs': runs}
    profile_sha256 = MODULE.canonical_profile_sha256(
        {'competitive_slam_profile': contract})
    if contract is CONTRACT:
        receipt_path = ROOT / 'configs' / 'slam_benchmark_profiles' / (
            'competitive_execution_selection_2026-08.yaml')
        receipt_sha = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
    else:
        receipt_path = ROOT / 'graph_based_slam' / 'test' / 'fixtures' / (
            'competitive_execution_selection_ready.yaml')
        receipt_sha = hashlib.sha256(receipt_path.read_bytes()).hexdigest()
    return {
        'schema_version': 2,
        'evidence_kind': 'competitive_slam_victory_evidence',
        'profile': contract['name'],
        'contract': {
            'datasets': datasets,
            'execution_selection_receipt_sha256': receipt_sha,
            'profile_sha256': profile_sha256,
            'profile_sha256_kind': MODULE.PROFILE_CANONICAL_HASH_KIND,
            'partitions': {
                'historical': {
                    'datasets': [slot['sequence'] for slot in historical_slots.values()],
                    'slots': copy.deepcopy(historical_slots),
                },
                'fresh': {
                    'datasets': [slot['sequence'] for slot in fresh_slots.values()],
                    'slots': copy.deepcopy(fresh_slots),
                },
            },
        },
        'systems': systems,
    }


def test_v2_synthetic_evidence_passes_with_cluster_bootstrap():
    evidence = _v2_evidence()
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'PASS'
    assert result['pass'] is True
    assert result['best_rival'] == 'glim'
    assert result['aggregate_ape_improvement_percent'] == pytest.approx(20.0)
    assert result['bootstrap_ci']['independent_unit'] == 'dataset'
    assert result['bootstrap_ci']['runs_treated_as_pseudo_independent'] is False
    assert result['bootstrap_ci']['superiority'] is True


def test_v2_rival_revision_must_match_profile_pin():
    evidence = _v2_evidence()
    evidence['systems']['glim']['provenance']['revision'] = 'd' * 40
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INVALID'
    assert result['pass'] is False
    assert result['checks']['profile_pinned_rival_revisions']['pass'] is False
    assert any('does not match profile.rivals.glim.revision' in error
               for error in result['errors'])


def test_v2_phase_flags_replace_wall_rtf_primary_gate():
    evidence = _v2_evidence()
    for system in evidence['systems'].values():
        for run in system['runs']:
            run['runtime'].update({
                'phase_contract_version': 'm6a10-online-compute-v2',
                'phase_mode': 'paced_1x',
                'paced_followability_passed': True,
                'unpaced_throughput_gate_passed': False,
            })
            run['runtime']['processing_rtf'] = 9.0
            run['runtime']['online_compute_rtf'] = 9.0
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['pass'] is True
    assert result['checks']['processing_rtf_leq_one']['evidence'][
        'role'] == 'diagnostic_only_for_v2'

    failed = copy.deepcopy(evidence)
    failed['systems']['ours']['runs'][0]['runtime'][
        'paced_followability_passed'] = False
    result = MODULE.evaluate_evidence_v2(failed, SYNTHETIC_CONTRACT)
    assert result['pass'] is False


def test_v2_missing_run_is_incomplete_and_cannot_pass():
    evidence = _v2_evidence()
    evidence['systems']['ours']['runs'].pop()
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INCOMPLETE'
    assert result['pass'] is False
    assert result['checks']['three_complete_runs_and_completion']['pass'] is False


def test_v2_missing_metric_is_incomplete_not_silently_omitted():
    evidence = _v2_evidence()
    del evidence['systems']['ours']['runs'][0]['trajectory']['ape_rmse_m']
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INCOMPLETE'
    assert result['pass'] is False


def test_v2_old_schema_is_fail_closed():
    result = MODULE.evaluate_evidence_v2({'schema_version': 1}, CONTRACT)
    assert result['status'] == 'INVALID'
    assert result['pass'] is False


def test_v2_failed_run_is_recorded_but_fails_gate():
    evidence = _v2_evidence()
    failed = evidence['systems']['glim']['runs'][0]
    failed.update({'complete': False, 'process_exit_status': 17,
                   'trajectory_complete': False})
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'FAIL'
    assert result['pass'] is False
    assert result['checks']['three_complete_runs_and_completion']['pass'] is False


def test_v2_exact_ten_percent_boundary_passes():
    result = MODULE.evaluate_evidence_v2(
        _v2_evidence(ours_ape=0.9, glim_ape=1.0, fast_ape=1.2),
        SYNTHETIC_CONTRACT)
    assert result['aggregate_ape_improvement_percent'] == pytest.approx(10.0)
    assert result['checks']['aggregate_ape_improvement']['pass'] is True
    assert result['pass'] is True


def test_v2_below_ten_percent_boundary_fails():
    result = MODULE.evaluate_evidence_v2(
        _v2_evidence(ours_ape=0.901, glim_ape=1.0, fast_ape=1.2),
        SYNTHETIC_CONTRACT)
    assert result['aggregate_ape_improvement_percent'] < 10.0
    assert result['checks']['aggregate_ape_improvement']['pass'] is False
    assert result['pass'] is False


def test_v2_rtf_and_common_identity_mismatch_fail_closed():
    evidence = _v2_evidence()
    evidence['systems']['ours']['runs'][0]['runtime']['processing_rtf'] = 1.01
    evidence['systems']['fast_livo2']['provenance']['thread_policy'] = {
        'omp_num_threads': 2, 'omp_dynamic': False}
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['pass'] is False
    assert result['checks']['processing_rtf_leq_one']['pass'] is False
    assert result['checks']['pinned_common_identity']['pass'] is False


def test_v2_frozen_slots_require_identity_and_ignore_self_declared_fresh_flag():
    assert all(
        slot['status'] == 'frozen_unopened'
        for slot in SYNTHETIC_CONTRACT['datasets']['fresh_holdout_slots'].values())
    # Keep this positive contract test independent from the repository's
    # deliberately stale/pending execution-selection checkpoint.
    evidence = _v2_evidence(SYNTHETIC_CONTRACT)
    evidence['contract']['fresh_holdout'] = True
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'PASS'
    assert result['pass'] is True
    assert result['checks']['fresh_slot_contract_not_self_declared']['pass'] is True
    assert result['checks']['execution_selection_receipt_registered']['pass'] is True


def test_v2_pending_execution_receipt_blocks_otherwise_perfect_fixture(
        tmp_path, monkeypatch):
    pending_contract = copy.deepcopy(SYNTHETIC_CONTRACT)
    evidence = _v2_evidence(pending_contract)
    pending_policy = pending_contract['evidence_gate_v2']
    pending_policy['execution_selection_receipt_path'] = (
        'configs/slam_benchmark_profiles/'
        'competitive_execution_selection_2026-08.yaml')
    receipt_path = (tmp_path / pending_policy['execution_selection_receipt_path'])
    receipt_path.parent.mkdir(parents=True)
    receipt = yaml.safe_load(
        (ROOT / 'graph_based_slam' / 'test' / 'fixtures' /
         'competitive_execution_selection_ready.yaml').read_text(
             encoding='utf-8'))
    receipt['common_identity']['profile_sha256'] = (
        MODULE.canonical_profile_sha256(
            {'competitive_slam_profile': pending_contract}))
    receipt['status'] = 'pending'
    receipt_path.write_text(yaml.safe_dump(receipt, sort_keys=False),
                            encoding='utf-8')
    pending_policy['execution_selection_receipt_sha256'] = hashlib.sha256(
        receipt_path.read_bytes()).hexdigest()
    evidence['contract']['execution_selection_receipt_sha256'] = (
        pending_policy['execution_selection_receipt_sha256'])
    evidence['contract']['profile_sha256'] = MODULE.canonical_profile_sha256(
        {'competitive_slam_profile': pending_contract})
    selection_source = ROOT / 'configs' / 'slam_benchmark_profiles' / (
        'competitive_execution_selection_2026-08-r2.yaml')
    selection_target = tmp_path / 'configs' / 'slam_benchmark_profiles' / (
        'competitive_execution_selection_2026-08-r2.yaml')
    selection_target.parent.mkdir(parents=True, exist_ok=True)
    selection_target.write_bytes(selection_source.read_bytes())
    monkeypatch.setattr(MODULE, 'ROOT', tmp_path)
    result = MODULE.evaluate_evidence_v2(evidence, pending_contract)
    assert result['status'] == 'INCOMPLETE'
    assert result['pass'] is False
    assert result['checks']['execution_selection_receipt_registered']['pass'] is False


def test_v2_dataset_identity_hash_mismatch_fails_closed():
    evidence = _v2_evidence()
    evidence['systems']['ours']['runs'][0]['dataset_identity'][
        'input_manifest_sha256'] = 'f' * 64
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INVALID'
    assert any('dataset identity mismatch' in error for error in result['errors'])


def _set_ape(evidence, system, dataset, values):
    runs = [run for run in evidence['systems'][system]['runs']
            if run['dataset'] == dataset]
    for run, value in zip(runs, values):
        run['trajectory']['ape_rmse_m'] = value


def test_v2_within_dataset_run_variance_can_fail_bootstrap_ci():
    evidence = _v2_evidence()
    _set_ape(evidence, 'ours', 'fresh_exp01', [0.1, 0.1, 0.1])
    _set_ape(evidence, 'glim', 'fresh_exp01', [1.0, 1.0, 1.0])
    _set_ape(evidence, 'ours', 'fresh_exp02', [1.8, 1.8, 1.8])
    _set_ape(evidence, 'glim', 'fresh_exp02', [1.0, 1.0, 1.0])
    _set_ape(evidence, 'ours', 'fresh_exp03', [0.1, 0.1, 0.1])
    _set_ape(evidence, 'glim', 'fresh_exp03', [1.0, 1.0, 1.0])
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['aggregate_ape_improvement']['pass'] is True
    assert result['bootstrap_ci_by_rival']['glim']['superiority'] is False
    assert result['checks']['dataset_cluster_bootstrap_95_superiority']['pass'] is False
    assert result['pass'] is False


def test_v2_single_sequence_primary_collapse_is_not_hidden_by_aggregate():
    evidence = _v2_evidence()
    _set_ape(evidence, 'ours', 'fresh_exp01', [1.5, 1.5, 1.5])
    _set_ape(evidence, 'ours', 'fresh_exp02', [0.5, 0.5, 0.5])
    _set_ape(evidence, 'ours', 'fresh_exp03', [0.5, 0.5, 0.5])
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['aggregate_ape_improvement']['pass'] is True
    assert result['checks']['per_sequence_primary_non_regression']['pass'] is False
    assert result['pass'] is False


def test_v2_map_regression_is_checked_per_dataset_and_rival():
    evidence = _v2_evidence()
    for run in evidence['systems']['ours']['runs']:
        if run['dataset'] == 'fresh_exp02':
            run['map']['plane_thickness_mean_m'] = 0.2
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    map_check = result['checks']['mapping_non_regression']['evidence']['comparisons']['glim']
    assert map_check['datasets']['fresh_exp02']['pass'] is False
    assert result['checks']['mapping_non_regression']['pass'] is False


def test_v2_invalid_fingerprint_is_rejected():
    evidence = _v2_evidence()
    evidence['systems']['ours']['provenance']['toolchain_fingerprint'] = 'gcc-release'
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INVALID'
    assert any('toolchain_fingerprint' in error for error in result['errors'])


def test_v2_all_rival_bootstrap_cis_are_required():
    evidence = _v2_evidence()
    _set_ape(evidence, 'ours', 'fresh_exp01', [0.1, 0.1, 0.1])
    _set_ape(evidence, 'fast_livo2', 'fresh_exp01', [0.9, 0.9, 0.9])
    _set_ape(evidence, 'ours', 'fresh_exp02', [1.8, 1.8, 1.8])
    _set_ape(evidence, 'fast_livo2', 'fresh_exp02', [1.1, 1.1, 1.1])
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert set(result['bootstrap_ci_by_rival']) == {'glim', 'fast_livo2'}
    assert result['checks']['dataset_cluster_bootstrap_95_superiority']['pass'] is False


def test_v2_historical_run_is_required_even_when_fresh_aggregate_passes():
    evidence = _v2_evidence()
    evidence['systems']['ours']['runs'] = [
        run for run in evidence['systems']['ours']['runs']
        if not (run['dataset'] == 'exp02' and run['run_index'] == 3)]
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INCOMPLETE'
    assert result['checks']['three_complete_runs_and_completion']['pass'] is False
    assert result['partitions']['historical']['included_in_aggregate_ape'] is False


def test_v2_historical_dataset_identity_hash_is_checked():
    evidence = _v2_evidence()
    historical_run = next(run for run in evidence['systems']['glim']['runs']
                          if run['dataset'] == 'exp02')
    historical_run['dataset_identity']['reference_sha256'] = 'f' * 64
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INVALID'
    assert any('dataset identity mismatch' in error for error in result['errors'])


def test_v2_historical_sequence_collapse_cannot_hide_behind_fresh_aggregate():
    evidence = _v2_evidence()
    _set_ape(evidence, 'ours', 'exp02', [1.5, 1.5, 1.5])
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['aggregate_ape_improvement']['pass'] is True
    assert result['checks']['per_sequence_primary_non_regression']['pass'] is False
    assert result['pass'] is False


def test_v2_historical_map_and_rtf_are_not_report_only():
    evidence = _v2_evidence()
    for run in evidence['systems']['ours']['runs']:
        if run['dataset'] == 'exp02':
            run['map']['plane_thickness_p95_m'] = 0.2
            run['runtime']['processing_rtf'] = 1.01
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['processing_rtf_leq_one']['pass'] is False
    assert result['checks']['mapping_non_regression']['pass'] is False


def test_v2_scorer_fingerprint_is_common_identity():
    evidence = _v2_evidence()
    evidence['systems']['fast_livo2']['provenance']['scorer_fingerprint'] = '4' * 64
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['scorer_fingerprint_common']['pass'] is False
    assert result['pass'] is False


def test_v2_thread_policy_requires_schema_and_canonical_equality():
    evidence = _v2_evidence()
    evidence['systems']['ours']['provenance']['thread_policy']['cpu_affinity'] = []
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['status'] == 'INVALID'
    assert result['checks']['thread_policy_contract']['pass'] is False

    evidence = _v2_evidence()
    evidence['systems']['glim']['provenance']['thread_policy']['max_threads'] = 4
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['thread_policy_contract']['pass'] is False


def test_v2_cli_writes_matching_json_and_yaml_receipts(tmp_path):
    evidence_path = tmp_path / 'evidence.yaml'
    profile_path = tmp_path / 'profile.yaml'
    output_path = tmp_path / 'receipt.json'
    yaml_output_path = tmp_path / 'receipt.yaml'
    evidence_path.write_text(yaml.safe_dump(_v2_evidence()), encoding='utf-8')
    profile_path.write_text(
        yaml.safe_dump({'competitive_slam_profile': SYNTHETIC_CONTRACT}),
        encoding='utf-8')
    command = [
        'python3', str(SCRIPT), '--evidence', str(evidence_path),
        '--profile', str(profile_path), '--output', str(output_path),
        '--yaml-output', str(yaml_output_path),
    ]
    completed = subprocess.run(command, check=False, capture_output=True,
                               text=True)
    assert completed.returncode == 0, completed.stderr
    json_receipt = json.loads(output_path.read_text(encoding='utf-8'))
    yaml_receipt = yaml.safe_load(yaml_output_path.read_text(encoding='utf-8'))
    assert json_receipt['status'] == 'PASS'
    assert yaml_receipt['status'] == json_receipt['status']
    assert len(json_receipt['receipt_identity']['profile_sha256']) == 64
    assert len(json_receipt['receipt_identity']['evidence_sha256']) == 64
    assert yaml_receipt['receipt_identity'] == json_receipt['receipt_identity']


def test_v2_minimal_cli_is_incomplete_but_writes_both_receipts(tmp_path):
    evidence_path = tmp_path / 'minimal.yaml'
    output_path = tmp_path / 'minimal.json'
    yaml_output_path = tmp_path / 'minimal-receipt.yaml'
    evidence_path.write_text(yaml.safe_dump({
        'schema_version': 2,
        'evidence_kind': 'competitive_slam_victory_evidence',
        'contract': {},
        'systems': {},
    }), encoding='utf-8')
    completed = subprocess.run([
        'python3', str(SCRIPT), '--evidence', str(evidence_path),
        '--profile', str(PROFILE), '--output', str(output_path),
        '--yaml-output', str(yaml_output_path),
    ], check=False, capture_output=True, text=True)
    assert completed.returncode == 1, completed.stderr
    json_receipt = json.loads(output_path.read_text(encoding='utf-8'))
    yaml_receipt = yaml.safe_load(yaml_output_path.read_text(encoding='utf-8'))
    assert json_receipt['status'] == 'INCOMPLETE'
    assert json_receipt['pass'] is False
    assert yaml_receipt['status'] == 'INCOMPLETE'
    assert yaml_receipt['pass'] is False


def test_v2_negative_receipts_are_json_and_yaml_serializable():
    pending = _v2_evidence()
    pending['contract']['fresh_holdout'] = True
    invalid_fingerprint = _v2_evidence()
    invalid_fingerprint['systems']['ours']['provenance'][
        'toolchain_fingerprint'] = 'not-a-fingerprint'
    missing_thread = _v2_evidence()
    missing_thread['systems']['glim']['provenance']['thread_policy'] = {}
    failed_run = _v2_evidence()
    failed_run['systems']['ours']['runs'][0].update({
        'complete': False, 'process_exit_status': 17,
        'trajectory_complete': False,
    })
    minimal = {
        'schema_version': 2,
        'evidence_kind': 'competitive_slam_victory_evidence',
        'contract': {},
        'systems': {},
    }
    results = [
        MODULE.evaluate_evidence_v2({'schema_version': 1}, CONTRACT),
        MODULE.evaluate_evidence_v2(minimal, CONTRACT),
        MODULE.evaluate_evidence_v2(pending, CONTRACT),
        MODULE.evaluate_evidence_v2(invalid_fingerprint, SYNTHETIC_CONTRACT),
        MODULE.evaluate_evidence_v2(missing_thread, SYNTHETIC_CONTRACT),
        MODULE.evaluate_evidence_v2(failed_run, SYNTHETIC_CONTRACT),
    ]
    for result in results:
        json.dumps(result)
        yaml.safe_dump(result)


def test_v2_claim_request_cannot_skip_per_run_resource_receipts():
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['pass'] is False
    assert result['memory_gate']['status'] == 'FAIL_CLOSED'
    assert result['checks']['memory_resource_receipt_gate']['pass'] is False
    assert any('resource_receipt_missing' in item
               for item in result['memory_gate']['errors'])


def test_v2_fresh_partition_reuse_of_historical_identity_fails_closed():
    contract = copy.deepcopy(SYNTHETIC_CONTRACT)
    fresh_input = contract['datasets']['fresh_holdout_slots']['fresh_1'][
        'input_manifest_sha256']
    contract['datasets']['holdout_slots']['holdout_1'][
        'input_manifest_sha256'] = fresh_input
    result = MODULE.evaluate_evidence_v2(_v2_evidence(contract), contract)
    assert result['checks']['fresh_partition_disjoint_immutable_identities'][
        'pass'] is False
    assert any('reuses an immutable dataset ID or hash' in error
               for error in result['errors'])


def test_v2_claim_request_requires_fresh_holdout_authorization_chain():
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['checks']['fresh_holdout_authorization']['pass'] is False
    assert result['checks']['dataset_source_closure']['pass'] is False
    assert result['status'] in {'INVALID', 'INCOMPLETE'}
    assert any('authorization mapping is missing' in error
               for error in result['errors'])


def test_v2_claim_request_reopens_execution_selection_preflight():
    """A claim cannot rely on receipt path/SHA alone.

    The synthetic fixture intentionally lacks the production execution
    identity closure.  The claim path must nevertheless invoke the full
    preflight checker, rather than treating the registered receipt as
    sufficient after only checking its file hash.
    """
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    preflight = result['checks']['execution_selection_preflight']
    assert preflight['pass'] is False
    assert preflight['evidence']['status'] in {'INVALID', 'INCOMPLETE'}
    assert any('execution selection preflight:' in error
               for error in result['errors'])


def test_v2_report_only_does_not_claim_execution_preflight():
    evidence = _v2_evidence()
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    preflight = result['checks']['execution_selection_preflight']
    assert preflight['pass'] is True
    assert preflight['evidence']['status'] == 'NOT_REQUESTED'
    assert preflight['evidence']['claim_eligible'] is False


def test_v2_claim_preflight_exception_is_fail_closed(monkeypatch):
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True

    def raise_unreopenable(*_args, **_kwargs):
        raise OSError('synthetic receipt disappeared during reopen')

    monkeypatch.setattr(MODULE, 'evaluate_execution_selection',
                        raise_unreopenable)
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    preflight = result['checks']['execution_selection_preflight']
    assert preflight['pass'] is False
    assert preflight['evidence']['status'] == 'FAIL_CLOSED'
    assert any('checker exception: synthetic receipt disappeared during reopen'
               in error for error in result['errors'])


def test_v2_claim_non_mapping_preflight_is_fail_closed(monkeypatch):
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    monkeypatch.setattr(MODULE, 'evaluate_execution_selection',
                        lambda *_args, **_kwargs: None)
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    preflight = result['checks']['execution_selection_preflight']
    assert preflight['pass'] is False
    assert preflight['evidence']['status'] == 'FAIL_CLOSED'
    assert any('non-mapping' in error for error in result['errors'])


def test_v2_claim_passes_exact_run_artifact_index_to_bundle_verifier(monkeypatch):
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    evidence['evidence_bundle'] = {
        'root': '/synthetic/bundle',
        'manifest_path': 'bundle_manifest.json',
    }
    captured = {}

    def capture_bundle(*_args, **kwargs):
        captured.update(kwargs)
        return {'status': 'FAIL_CLOSED', 'pass': False,
                'claim_eligible': False, 'errors': ['synthetic bundle stop']}

    monkeypatch.setattr(MODULE, 'verify_evidence_bundle', capture_bundle)
    MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    bindings = captured.get('expected_run_artifacts')
    assert isinstance(bindings, list)
    expected_count = sum(
        len(record['runs']) for record in evidence['systems'].values())
    assert len(bindings) == expected_count
    identities = {
        (item['system'], item['dataset'], item['run_index'])
        for item in bindings
    }
    assert len(identities) == expected_count
    assert all(len(item[field]) == 64 for item in bindings
               for field in ('trajectory_sha256', 'map_sha256',
                             'score_sha256', 'execution_receipt_sha256',
                             'execution_receipt_file_sha256', 'campaign_id'))
    # The synthetic fixture intentionally has no RSS receipts.  Passing that
    # absence through as None makes the claim bundle checker fail closed rather
    # than silently treating a score-only run as resource-bound.
    assert all(item['resource_sha256'] is None for item in bindings)


def _claim_requested_synthetic_evidence():
    evidence = _v2_evidence()
    evidence['claim_eligible'] = True
    evidence['evidence_bundle'] = {
        'root': '/synthetic/bundle',
        'manifest_path': 'bundle_manifest.json',
    }
    return evidence


def test_v2_claim_rejects_cloned_execution_receipt_for_relabelled_run():
    evidence = _claim_requested_synthetic_evidence()
    runs = evidence['systems']['ours']['runs']
    runs[1]['execution_receipt_sha256'] = runs[0]['execution_receipt_sha256']
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['pass'] is False
    assert any('duplicate execution receipt identity' in error
               for error in result['errors'])


def test_v2_claim_rejects_mixed_execution_campaigns():
    evidence = _claim_requested_synthetic_evidence()
    runs = evidence['systems']['ours']['runs']
    runs[1]['campaign_id'] = hashlib.sha256(
        b'a-different-campaign').hexdigest()
    result = MODULE.evaluate_evidence_v2(evidence, SYNTHETIC_CONTRACT)
    assert result['pass'] is False
    assert any('mixed execution campaign identities' in error
               for error in result['errors'])


def test_v2_score_artifact_digest_changes_with_scored_metric():
    run = copy.deepcopy(_v2_evidence()['systems']['ours']['runs'][0])
    first = MODULE._v2_score_artifact_sha256('ours', run)
    run['trajectory']['ape_rmse_m'] += 0.001
    second = MODULE._v2_score_artifact_sha256('ours', run)
    assert first != second
    assert len(first) == 64
    assert len(second) == 64
def test_formal_profile_has_all_holdout_inputs_frozen():
    result = MODULE.evaluate(_gates(), CONTRACT)
    assert result['checks']['all_holdout_inputs_frozen'] is True
