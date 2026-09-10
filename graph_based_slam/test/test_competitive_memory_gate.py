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


"""Adversarial synthetic tests for the competitive process-RSS gate."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'competitive_memory_gate.py'
SPEC = importlib.util.spec_from_file_location('competitive_memory_gate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _policy(*, per_sequence_status='READY', ceiling=1.20):
    return {
        'schema_version': 1,
        'source_kind': MODULE.SOURCE_KIND,
        'measurement_version': MODULE.MEASUREMENT_VERSION,
        'measurement_scope': MODULE.MEASUREMENT_SCOPE,
        'primary_metric': MODULE.PRIMARY_METRIC,
        'primary_metric_definition': MODULE.PRIMARY_METRIC_DEFINITION,
        'max_peak_rss_ratio_vs_best_rival': 1.20,
        'minimum_matched_complete_runs': 3,
        'aggregate': {
            'rival_selection': 'best_rival',
            'compare_every_pinned_rival': True,
            'threshold_source': 'max_peak_rss_ratio_vs_best_rival',
            'aggregation': 'max_peak_over_repetitions_and_sequences_v1',
        },
        'per_sequence': {
            'status': per_sequence_status,
            'serious_regression_ceiling_ratio': ceiling,
            'threshold_source': 'max_peak_rss_ratio_vs_best_rival',
        },
        'uncertainty': {
            'status': 'NOT_APPLICABLE', 'required': False,
            'scope': 'APE_ONLY', 'rss_ci_claim': False, 'method': None,
        },
        'resource_identity': {
            'tool_revision': 'm6a7-container-process-rss-v1',
            'sampler_script_sha256': '1' * 64,
            'memory_helper_script_sha256': '2' * 64,
        },
    }


THREAD_POLICY = {
    'cpu_affinity': [0, 1],
    'max_threads': 2,
    'omp_num_threads': 1,
    'openblas_num_threads': 1,
    'mkl_num_threads': 1,
    'tbb_num_threads': 1,
    'accelerator_policy': 'cpu',
}
THREAD_HASH = MODULE.canonical_json_sha256(THREAD_POLICY)
HARDWARE = '3' * 64
MACHINE = 'fixture-machine'


def _resource(system, sequence, index, peak, *, receipt_sha=None):
    identity = {
        'tool_revision': 'm6a7-container-process-rss-v1',
        'sampler_script_sha256': '1' * 64,
        'memory_helper_script_sha256': '2' * 64,
        'config_sha256': hashlib.sha256(
            f'config-{system}'.encode()).hexdigest(),
        'thread_policy_sha256': THREAD_HASH,
        'hardware_fingerprint': HARDWARE,
        'machine_id': MACHINE,
        'release': 'Release',
    }
    receipt = {
        'status': 'pass',
        'measurement_version': MODULE.MEASUREMENT_VERSION,
        'measurement_scope': MODULE.MEASUREMENT_SCOPE,
        'primary_metric': MODULE.PRIMARY_METRIC,
        'primary_metric_definition': MODULE.PRIMARY_METRIC_DEFINITION,
        MODULE.PRIMARY_METRIC: peak,
        'identity': copy.deepcopy(identity),
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'cgroup_events': {'oom_free': True, 'oom_delta': {'oom': 0}},
    }
    return {
        'status': 'PASS',
        'source_kind': MODULE.SOURCE_KIND,
        'receipt_sha256': receipt_sha or hashlib.sha256(
            f'{system}-{sequence}-{index}'.encode()).hexdigest(),
        'identity': identity,
        MODULE.PRIMARY_METRIC: peak,
        'receipt': receipt,
    }


def _run(system, sequence, index, peak, *, receipt_sha=None):
    peak_mb = peak / (1024.0 * 1024.0)
    return {
        'dataset': sequence,
        'run_index': index,
        'complete': True,
        'process_exit_status': 0,
        'trajectory_complete': True,
        'sequence_failure': False,
        'catastrophic_failure': False,
        'verified_false_loops': 0,
        'runtime': {'peak_rss_mb': peak_mb},
        'resource_evidence': _resource(
            system, sequence, index, peak, receipt_sha=receipt_sha),
    }


def _rows(*, ours=(100, 110), glim=(120, 115), fast=(125, 118), sequences=('s1', 's2')):
    values = {'ours': ours, 'glim': glim, 'fast_livo2': fast}
    result = {system: [] for system in values}
    for system, sequence_values in values.items():
        for sequence, value in zip(sequences, sequence_values):
            for index in (1, 2, 3):
                result[system].append(_run(
                    system, sequence, index, value * 1024 * 1024 + index))
    return result


def _evaluate(rows, *, policy=None, claim_requested=True, best_rival='glim'):
    return MODULE.evaluate_memory_gate(
        rows,
        required_systems=['ours', 'glim', 'fast_livo2'],
        expected_sequences=['s1', 's2'],
        repetitions=3,
        policy=policy or _policy(),
        best_rival=best_rival,
        claim_requested=claim_requested,
    )


def test_all_rivals_and_matched_runs_are_compared_deterministically():
    result = _evaluate(_rows())
    assert result['pass'] is True
    assert result['checks']['matched_run_conditions']['pass'] is True
    comparisons = result['checks']['aggregate_best_rival']['comparisons']
    assert set(comparisons) == {'glim', 'fast_livo2'}
    assert result['aggregate_peak_rss_bytes']['ours'] == 110 * 1024 * 1024 + 3
    assert result['checks']['per_sequence_serious_regression']['pass'] is True


def test_missing_run_and_missing_receipt_fail_closed():
    rows = _rows()
    rows['ours'].pop()
    result = _evaluate(rows)
    assert result['pass'] is False
    assert result['checks']['matched_run_conditions']['pass'] is False
    assert any('missing resource runs' in error for error in result['errors'])

    rows = _rows()
    rows['glim'][0].pop('resource_evidence')
    result = _evaluate(rows)
    assert result['pass'] is False
    assert any('resource_receipt_missing' in error for error in result['errors'])


@pytest.mark.parametrize('mutator,needle', [
    (lambda row: row['resource_evidence'].__setitem__(
        MODULE.PRIMARY_METRIC, 0), 'positive integer'),
    (lambda row: row['runtime'].__setitem__('peak_rss_mb', float('nan')),
     'finite'),
    (lambda row: row['resource_evidence']['receipt'].__setitem__(
        MODULE.PRIMARY_METRIC, 999), 'different bytes'),
])
def test_zero_nonfinite_and_duplicate_metric_values_are_rejected(mutator, needle):
    rows = _rows()
    mutator(rows['ours'][0])
    result = _evaluate(rows)
    assert result['pass'] is False
    assert any(needle in error for error in result['errors'])


def test_duplicate_receipt_identity_is_rejected():
    rows = _rows()
    reused = rows['ours'][0]['resource_evidence']['receipt_sha256']
    rows['ours'][1]['resource_evidence']['receipt_sha256'] = reused
    result = _evaluate(rows)
    assert result['pass'] is False
    assert any('duplicate resource receipt identity' in error
               for error in result['errors'])


def test_functional_only_plugin_resource_cannot_enter_gate():
    rows = _rows()
    resource = rows['ours'][0]['resource_evidence']
    resource['source_kind'] = 'functional_non_authoritative_v1'
    resource['functional_only'] = True
    result = _evaluate(rows)
    assert result['pass'] is False
    assert any('functional' in error or 'authoritative' in error
               for error in result['errors'])


@pytest.mark.parametrize('field', [
    'tool_revision', 'config_sha256', 'thread_policy_sha256',
    'hardware_fingerprint', 'machine_id', 'release',
])
def test_resource_identity_drift_is_rejected(field):
    rows = _rows()
    identity = rows['fast_livo2'][0]['resource_evidence']['identity']
    if field == 'tool_revision':
        identity[field] = 'm6a7-container-process-rss-v2'
    elif field == 'machine_id':
        identity[field] = 'other-machine'
    elif field == 'config_sha256':
        identity[field] = '4' * 64
    elif field == 'release':
        identity[field] = 'Debug'
    else:
        identity[field] = '4' * 64
    rows['fast_livo2'][0]['resource_evidence']['receipt']['identity'][field] = (
        identity[field])
    result = _evaluate(rows)
    assert result['pass'] is False
    assert result['errors']


def test_every_pinned_rival_must_meet_aggregate_ratio():
    rows = _rows(fast=(50, 50))
    result = _evaluate(rows)
    assert result['pass'] is False
    assert result['checks']['aggregate_best_rival']['comparisons']['fast_livo2']['pass'] is False


def test_profile_binds_ready_per_sequence_ceiling_to_canonical_ratio():
    import copy

    profile = yaml.safe_load((ROOT / 'configs/slam_benchmark_profiles' /
                              'competitive_slam_v1.yaml').read_text())
    policy = copy.deepcopy(
        profile['competitive_slam_profile']['evidence_gate_v2']['memory_gate'])
    # The synthetic receipt uses fixture source hashes; retain the profile's
    # policy shape while keeping the fixture entirely synthetic.
    policy['resource_identity']['sampler_script_sha256'] = '1' * 64
    policy['resource_identity']['memory_helper_script_sha256'] = '2' * 64
    result = _evaluate(_rows(), policy=policy)
    assert result['pass'] is True
    assert result['status'] == 'PASS'
    sequence_check = result['checks']['per_sequence_serious_regression']
    assert sequence_check['status'] == 'READY'
    assert sequence_check['threshold_source'] == (
        'max_peak_rss_ratio_vs_best_rival')


def test_less_than_three_matched_runs_is_rejected():
    rows = _rows()
    for system in rows:
        rows[system].pop()
    result = MODULE.evaluate_memory_gate(
        rows,
        required_systems=['ours', 'glim', 'fast_livo2'],
        expected_sequences=['s1', 's2'],
        repetitions=2,
        policy=_policy(),
        best_rival='glim',
        claim_requested=True,
    )
    assert result['pass'] is False
    assert any('at least three' in error for error in result['errors'])


def test_per_sequence_ratio_ceiling_prevents_aggregate_hiding_regression():
    result = _evaluate(_rows(ours=(130, 100), glim=(100, 100), fast=(110, 110)))
    assert result['pass'] is False
    row = result['checks']['per_sequence_serious_regression']['sequences']['s1']
    assert row['comparisons']['glim']['ratio'] == pytest.approx(1.30)
    assert row['comparisons']['glim']['pass'] is False


def test_required_but_unconfigured_rss_uncertainty_fails_closed():
    policy = _policy()
    policy['uncertainty'] = {
        'status': 'NOT_CONFIGURED', 'required': True, 'method': None,
    }
    result = _evaluate(_rows(), policy=policy)
    assert result['pass'] is False
    assert result['checks']['uncertainty']['pass'] is False
    assert any('RSS uncertainty/CI' in error for error in result['errors'])


def test_claim_request_without_resources_cannot_upgrade_report_only_data():
    rows = {system: [] for system in ('ours', 'glim', 'fast_livo2')}
    report_only = _evaluate(rows, claim_requested=False)
    assert report_only['status'] == 'NOT_REQUESTED'
    assert report_only['pass'] is True
    claim = _evaluate(rows, claim_requested=True)
    assert claim['pass'] is False
