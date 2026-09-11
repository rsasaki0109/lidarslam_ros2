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

"""Static, GT-free contract checks for the NTU public training input."""

import json
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_repo_profile_names_public_ntu_training_input():
    profile = yaml.safe_load(
        (ROOT / 'scripts' / 'release_profiles.yaml').read_text())
    rows = [row for row in profile['release_profiles']
            if row.get('name') == 'ntu_viral_tnp_01']
    assert len(rows) == 1
    assert rows[0]['match']['bag_name_contains'] == 'tnp_01'
    assert rows[0]['match']['reference_kind'] == 'ground_truth'


def test_ntu_glim_override_matches_canonical_topics():
    config = json.loads((ROOT / 'configs' / 'glim' / 'ntu_viral_cpu' /
                         'config_ros.json').read_text())['glim_ros']
    assert config['points_topic'] == '/os1_cloud_node1/points'
    assert config['imu_topic'] == '/imu/imu'
    assert config['image_topic'] == '/left/image_raw'


def test_wrappers_expose_profile_selection_without_changing_default():
    ours = (ROOT / 'scripts' / 'ours_container_gt_blind_run.sh').read_text()
    glim = (ROOT / 'scripts' / 'glim_container_run.sh').read_text()
    fast = (ROOT / 'scripts' / 'fast_livo2_container_run.sh').read_text()
    assert 'RKO_CONFIG="${RKO_PARAM:-' in ours
    assert 'GLIM_PROFILE="${GLIM_PROFILE:-hilti2022_cpu}"' in glim
    assert 'FAST_PROFILE="${FAST_PROFILE:-hilti22}"' in fast
    assert 'mapping_ouster_ntu.launch' in fast


def test_training_input_has_no_gt_or_campaign4_contract_in_repo_paths():
    paths = list((ROOT / 'configs' / 'glim' / 'ntu_viral_cpu').glob('*.json'))
    paths.append(ROOT / 'lidarslam' / 'param' / 'rko_lio_ntu_viral.yaml')
    for path in paths:
        assert 'ground_truth' not in path.read_text().lower()
        assert 'campaign4' not in path.read_text().lower()


def test_portable_resource_report_is_gnu_time_compatible(tmp_path):
    report = tmp_path / 'compute_time.txt'
    helper = ROOT / 'scripts' / 'portable_resource_time.py'
    subprocess.run(
        [
            sys.executable,
            str(helper),
            '--output',
            str(report),
            '--',
            sys.executable,
            '-c',
            'print("training-smoke")',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    text = report.read_text()
    assert 'Elapsed (wall clock) time (seconds):' in text
    assert 'Maximum resident set size (kbytes):' in text
    assert 'File system inputs:' in text
    assert 'File system outputs:' in text
    assert 'Exit status: 0' in text


def test_ros1_phase_parser_skips_headers_and_normalizes_nanoseconds():
    parser = (ROOT / 'scripts' / 'container_phase_evidence.sh').read_text()
    assert 'text.startswith("%")' in parser
    assert 'value /= 1.0e9' in parser


def test_fast_probe_is_bounded_and_keeps_loopback_only_contract():
    wrapper = (ROOT / 'scripts' / 'fast_livo2_container_run.sh').read_text()
    driver = (ROOT / 'scripts' / 'run_competitive_gt_blind_benchmark.py').read_text()
    assert 'timeout 10 rosnode ping' in wrapper
    assert 'ROS_MASTER_URI=http://127.0.0.1:11311' in wrapper
    assert "'--network', 'none'" in driver


def test_v2_phase_bridge_requires_application_consumer_evidence():
    bridge = (ROOT / 'scripts' / 'container_phase_evidence.sh').read_text()
    profile = (ROOT / 'configs' / 'slam_benchmark_profiles' /
               'competitive_slam_v1.yaml').read_text()
    assert 'M6A10_CONSUMER_EVIDENCE' in bridge
    assert 'publisher_count_cannot_prove_consumer_processing' in (
        ROOT / 'scripts' / 'benchmark_phase_contract.py').read_text()
    assert 'phase_contract_v2:' in profile
    assert 'maximum_callback_latency_seconds: 0.25' in profile
    assert 'require_ack_backpressure_for_unpaced: true' in profile
