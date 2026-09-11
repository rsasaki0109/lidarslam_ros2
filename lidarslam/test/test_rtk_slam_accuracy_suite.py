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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the declarative RTK-SLAM accuracy suite."""

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'run_rtk_slam_accuracy_suite.py'
SPEC = importlib.util.spec_from_file_location('rtk_slam_accuracy_suite', SCRIPT)
SUITE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(SUITE)


def test_contract_covers_all_sequences_with_measured_completion_behavior():
    contract = SUITE.load_contract()
    sequences = contract['sequences']

    assert list(sequences) == [
        'construction_seq2',
        'construction_seq1',
        'stadtgarten_seq2',
        'stadtgarten_seq1',
    ]
    assert sequences['construction_seq2']['completion_end_margin_secs'] == 0.25
    assert sequences['construction_seq1']['completion_end_margin_secs'] == 0.25
    assert sequences['stadtgarten_seq1']['completion_end_margin_secs'] == 2
    assert sequences['stadtgarten_seq2']['completion_end_margin_secs'] == 65
    assert sequences['stadtgarten_seq2']['rko_param'].endswith(
        'rko_lio_rtk_slam_mid360_stadtgarten_seq2.yaml')
    assert sequences['stadtgarten_seq1']['rko_param'].endswith(
        'rko_lio_rtk_slam_mid360_outdoor.yaml')


def test_default_and_all_sequence_selection():
    contract = SUITE.load_contract()

    assert SUITE.selected_sequences([], contract) == ['construction_seq2']
    assert SUITE.selected_sequences(['all'], contract) == list(
        contract['sequences'])


def test_sequence_selection_rejects_unknown_duplicates_and_ambiguous_all():
    contract = SUITE.load_contract()

    with pytest.raises(ValueError, match='unknown sequence'):
        SUITE.selected_sequences(['not_a_sequence'], contract)
    with pytest.raises(ValueError, match='duplicate'):
        SUITE.selected_sequences(
            ['construction_seq2', 'construction_seq2'], contract)
    with pytest.raises(ValueError, match='cannot be combined'):
        SUITE.selected_sequences(['all', 'construction_seq2'], contract)


def test_input_preflight_reports_all_missing_paths(tmp_path):
    contract = SUITE.load_contract()
    sequence = contract['sequences']['construction_seq2']

    with pytest.raises(ValueError) as raised:
        SUITE.resolve_inputs(tmp_path, 'construction_seq2', sequence, contract)

    message = str(raised.value)
    assert 'ROS2 bag metadata' in message
    assert 'checkpoints' in message


def test_commands_freeze_sensor_reference_and_sequence_parameters(tmp_path):
    contract = SUITE.load_contract()
    sequence = contract['sequences']['stadtgarten_seq2']
    paths = {
        'bag': tmp_path / 'bag',
        'checkpoints': tmp_path / 'checkpoints.csv',
        'rko_param': REPO_ROOT / sequence['rko_param'],
        'lidarslam_param': REPO_ROOT / contract['lidarslam_param'],
    }

    generate, benchmark = SUITE.commands_for(
        'stadtgarten_seq2', sequence, contract, paths, tmp_path / 'out', False)

    assert '--sequence' in generate
    assert generate[generate.index('--sequence') + 1] == 'stadtgarten_seq2'
    assert benchmark[benchmark.index('--lidar-topic') + 1] == '/livox/points'
    assert benchmark[benchmark.index('--imu-topic') + 1] == '/livox/imu'
    assert benchmark[benchmark.index('--base-frame') + 1] == 'os_sensor'
    assert benchmark[
        benchmark.index('--completion-end-margin-secs') + 1] == '65'
    assert benchmark[
        benchmark.index('--reference-source') + 1] == \
        'rtk_slam_stadtgarten_seq2_gt'
    assert '--skip-reference-gen' in benchmark
    assert '--skip-map-save' in benchmark


def test_save_maps_only_removes_skip_map_save(tmp_path):
    contract = SUITE.load_contract()
    sequence = contract['sequences']['construction_seq2']
    paths = {
        'bag': tmp_path / 'bag',
        'checkpoints': tmp_path / 'checkpoints.csv',
        'rko_param': REPO_ROOT / sequence['rko_param'],
        'lidarslam_param': REPO_ROOT / contract['lidarslam_param'],
    }

    _, benchmark = SUITE.commands_for(
        'construction_seq2', sequence, contract, paths, tmp_path / 'out', True)

    assert '--skip-map-save' not in benchmark
