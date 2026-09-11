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


"""Tests for composing heterogeneous runner artifacts into gate results."""

import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'compose_competitive_result.py'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'
SPEC = importlib.util.spec_from_file_location('competitive_result', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write(path, document):
    path.write_text(json.dumps(document))
    return path


def _inputs(tmp_path, map_valid=True):
    calibration = 'c' * 64
    manifest = _write(tmp_path / 'manifest.json', {
        'status': 'frozen', 'sequence': 'exp03',
        'hashes': {'calibration_sha256': calibration}})
    machine = _write(tmp_path / 'machine.json', {'machine_id': 'a' * 64})
    reference = tmp_path / 'reference.tum'
    reference.write_text('1 0 0 0 0 0 0 1\n')
    trajectory = _write(tmp_path / 'trajectory.json', {
        'valid_repetitions': 3,
        'reference': {'common_sha256': 'd' * 64},
        'aggregate': {'ape_rmse_median_m': 0.5},
        'runs': [
            {'completion': {'trajectory_complete': True,
                            'process_exit_status': 0},
             'runtime': {'processing_realtime_factor': value,
                         'online_compute_rtf': value - 0.1,
                         'wall_realtime_factor': value + 0.2,
                         'peak_rss_mb': rss}}
            for value, rss in ((0.8, 100), (0.9, 110), (0.7, 105))],
    })
    mapping = _write(tmp_path / 'map.json', {
        'valid_repetitions': 3,
        'meaningful_repetitions': 3 if map_valid else 2,
        'aggregation_valid': map_valid,
        'aggregate': ({'plane_thickness_mean_worst_m': 0.04,
                       'plane_thickness_p95_worst_m': 0.08,
                       'planar_coverage_worst': 0.6} if map_valid else None),
    })
    return manifest, reference, machine, trajectory, mapping


def test_compose_preserves_identity_and_conservative_aggregates(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(tmp_path)
    result = MODULE.compose(
        system='ours', track='glim_cpu_lidar_imu',
        manifest_path=manifest, reference_path=reference,
        machine_path=machine, trajectory_path=trajectory,
        map_path=mapping, profile_path=PROFILE)
    assert result['sequence'] == 'exp03'
    assert result['machine_id'] == 'a' * 64
    assert result['repetitions'] == {'valid': 3, 'failures': 0}
    assert result['runtime']['processing_rtf_median'] == 0.8
    assert result['runtime']['online_compute_rtf_median'] == pytest.approx(0.7)
    assert result['runtime']['wall_realtime_factor_median'] == 1.0
    assert result['runtime']['peak_rss_max_mb'] == 110
    assert result['mapping']['aggregation_valid'] is True


def test_invalid_map_and_invalid_processing_remain_invalid(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(
        tmp_path, map_valid=False)
    processing = _write(tmp_path / 'processing.json', {
        'valid_processing_rtf_evidence': False,
        'processing_rtf_upper_bound_median': 0.9})
    result = MODULE.compose(
        system='fast_livo2', track='fast_livo2_lidar_imu_visual',
        manifest_path=manifest, reference_path=reference,
        machine_path=machine, trajectory_path=trajectory,
        map_path=mapping, profile_path=PROFILE,
        processing_path=processing)
    assert result['mapping'] == {
        'aggregation_valid': False,
        'valid_repetitions': 3,
        'meaningful_repetitions': 2,
    }
    assert 'processing_rtf_median' not in result['runtime']


def test_unfrozen_manifest_is_rejected(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(tmp_path)
    manifest.write_text(json.dumps({'status': 'pending'}))
    try:
        MODULE.compose(
            system='ours', track='glim_cpu_lidar_imu',
            manifest_path=manifest, reference_path=reference,
            machine_path=machine, trajectory_path=trajectory,
            map_path=mapping, profile_path=PROFILE)
    except ValueError as error:
        assert 'not frozen' in str(error)
    else:
        raise AssertionError('unfrozen manifest was accepted')


def test_compose_propagates_two_layer_execution_identity(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(tmp_path)
    document = json.loads(trajectory.read_text())
    for index, run in enumerate(document['runs'], 1):
        run.update({
            'campaign_id': 'c' * 64,
            'execution_receipt_sha256': ('a' * 63) + str(index),
            'execution_receipt_file_sha256': ('b' * 63) + str(index),
        })
    trajectory.write_text(json.dumps(document))
def test_legacy_calibration_archive_hash_remains_explicitly_supported(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(tmp_path)
    document = json.loads(manifest.read_text())
    value = document['hashes'].pop('calibration_sha256')
    document['hashes']['calibration_archive_sha256'] = value
    manifest.write_text(json.dumps(document))
    result = MODULE.compose(
        system='ours', track='glim_cpu_lidar_imu',
        manifest_path=manifest, reference_path=reference,
        machine_path=machine, trajectory_path=trajectory,
        map_path=mapping, profile_path=PROFILE)
    assert len(result['execution_evidence']) == 3
    assert result['execution_evidence'][0]['execution_receipt_sha256'].endswith('1')
    assert result['execution_evidence'][0]['execution_receipt_file_sha256'].endswith('1')
    assert result['calibration_sha256'] == value


def test_fast_mapper_nested_peak_rss_is_used_conservatively(tmp_path):
    manifest, reference, machine, trajectory, mapping = _inputs(tmp_path)
    document = json.loads(trajectory.read_text())
    for run, rss in zip(document['runs'], (120, 140, 130)):
        run['runtime'].pop('peak_rss_mb')
        run['runtime']['mapper'] = {'peak_rss_mb': rss}
    trajectory.write_text(json.dumps(document))
    processing = _write(tmp_path / 'processing.json', {
        'valid_processing_rtf_evidence': True,
        'processing_rtf_upper_bound_median': 0.95})
    result = MODULE.compose(
        system='fast_livo2', track='fast_livo2_lidar_imu_visual',
        manifest_path=manifest, reference_path=reference,
        machine_path=machine, trajectory_path=trajectory,
        map_path=mapping, profile_path=PROFILE,
        processing_path=processing)
    assert result['runtime'] == {
        'peak_rss_max_mb': 140.0, 'processing_rtf_median': 0.95}
