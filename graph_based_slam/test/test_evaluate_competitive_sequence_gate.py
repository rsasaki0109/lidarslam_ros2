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


"""Tests for the all-or-nothing competitive sequence gate."""

import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'evaluate_competitive_sequence_gate.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SPEC = importlib.util.spec_from_file_location('competitive_gate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)
CONTRACT = yaml.safe_load(PROFILE.read_text())['competitive_slam_profile']


def _result(system, track='glim_cpu_lidar_imu'):
    closure_identity = MODULE.current_rival_source_closure_identity(
        {'competitive_slam_profile': CONTRACT}, root=ROOT)
    return {
        'system': system, 'sequence': 'exp99', 'track': track,
        'input_manifest_sha256': 'a' * 64, 'reference_sha256': 'b' * 64,
        'calibration_sha256': 'c' * 64, 'machine_id': 'machine',
        'rival_source_closure': closure_identity,
        'excluded_capabilities': CONTRACT['excluded_capabilities'],
        'repetitions': {'valid': 3, 'failures': 0},
        'trajectory': {'ape_rmse_median_m': 0.09 if system == 'ours' else 0.1},
        'runtime': {'processing_rtf_median': 0.9,
                    'online_compute_rtf_median': 0.8,
                    'peak_rss_max_mb': 110},
        'mapping': {'aggregation_valid': True,
                    'valid_repetitions': 3,
                    'meaningful_repetitions': 3,
                    'plane_thickness_mean_worst_m': 0.04,
                    'plane_thickness_p95_worst_m': 0.08,
                    'planar_coverage_worst': 0.6},
        'loop_closure': {'verified_false_edges': 0},
    }


def test_all_required_dimensions_must_pass_together():
    ours, rival = _result('ours'), _result('rival')
    rival['runtime']['peak_rss_max_mb'] = 100
    result = MODULE.evaluate(ours, rival, CONTRACT)
    assert result['pass'] is True
    assert all(row['pass'] for row in result['checks'].values())


def test_accuracy_win_cannot_hide_map_or_identity_regression():
    ours, rival = _result('ours'), _result('rival')
    ours['trajectory']['ape_rmse_median_m'] = 0.01
    ours['mapping']['planar_coverage_worst'] = 0.3
    ours['input_manifest_sha256'] = 'different'
    result = MODULE.evaluate(ours, rival, CONTRACT)
    assert result['checks']['primary_accuracy_improvement']['pass'] is True
    assert result['checks']['mapping_non_regression']['pass'] is False
    assert result['checks']['identical_evaluation_contract']['pass'] is False
    assert result['pass'] is False


def test_visual_track_adds_heldout_colour_non_regression():
    ours = _result('ours', 'fast_livo2_lidar_imu_visual')
    rival = _result('rival', 'fast_livo2_lidar_imu_visual')
    ours['visual'] = {'heldout_rgb_l2_median': 40,
                      'heldout_rgb_inlier_20': 0.3}
    rival['visual'] = {'heldout_rgb_l2_median': 30,
                       'heldout_rgb_inlier_20': 0.4}
    result = MODULE.evaluate(ours, rival, CONTRACT)
    assert result['checks']['visual_colour_non_regression']['pass'] is False
    assert result['pass'] is False


def test_complete_but_non_meaningful_rival_map_is_a_quality_failure():
    ours, rival = _result('ours'), _result('rival')
    rival['mapping'] = {
        'aggregation_valid': False,
        'valid_repetitions': 3,
        'meaningful_repetitions': 2,
    }
    result = MODULE.evaluate(ours, rival, CONTRACT)
    mapping = result['checks']['mapping_non_regression']
    assert mapping['pass'] is True
    assert 'rival map evidence is complete' in mapping['evidence']['reason']


def test_missing_rival_map_or_non_meaningful_ours_map_fails():
    ours, rival = _result('ours'), _result('rival')
    rival['mapping']['valid_repetitions'] = 2
    result = MODULE.evaluate(ours, rival, CONTRACT)
    assert result['checks']['mapping_non_regression']['pass'] is False

    rival = _result('rival')
    ours['mapping']['aggregation_valid'] = False
    ours['mapping']['meaningful_repetitions'] = 2
    result = MODULE.evaluate(ours, rival, CONTRACT)
    assert result['checks']['mapping_non_regression']['pass'] is False
