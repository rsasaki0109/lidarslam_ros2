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


"""
Static gates for the additive v15 feeder image lineage.

These tests never invoke Docker, ROS, a bag, GT, scorer, or formal replay.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = ROOT / 'docker/fast_livo2_m6a10_v15.Dockerfile'
BUILD = ROOT / 'scripts/build_fast_livo2_m6a10_v15_image.sh'
WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v15_formal_container_run.sh'
PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v15_formal_ready.yaml'
FEEDER = ROOT / 'scripts/fast_livo2_m6a10_v15_feeder.py'
LEGACY = ROOT / 'scripts/fast_livo2_m6a10_feeder.py'
FIXTURE = ROOT / 'graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json'
BASE_TAG = 'm6a10-v2c-v12-nonlidar-boundary-transport-20260824-fast-livo2-benchmark:ros1-pinned'
BASE_ID = 'sha256:03dfa4c3e7c3f1ea9160ba2276ea23bfbdef43d441bc8afc628f907bd50743a7'
FEEDER_SHA = '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7'
LEGACY_SHA = '869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf'
PROFILE_SHA = '070fb2b762881a7126caf021c7a98ab59c8fa55e10be220b6482d7f2ef97899f'
PHASE = 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
TRANSPORT = 'm6a10-v12-callback-ack-transport-outstanding-v1'


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_v15_source_and_immutable_v12_lineage_hashes():
    assert _sha(FEEDER) == FEEDER_SHA
    assert _sha(LEGACY) == LEGACY_SHA
    assert _sha(PROFILE) == PROFILE_SHA
    assert DOCKERFILE.is_file() and BUILD.is_file() and WRAPPER.is_file()
    assert WRAPPER.stat().st_mode & 0o111


def test_v15_wrapper_pins_new_feeder_and_preserves_v5_contract():
    text = WRAPPER.read_text(encoding='utf-8')
    assert "EXPECTED_FEEDER_SHA256='%s'" % FEEDER_SHA in text
    assert "EXPECTED_PROFILE_SHA256='%s'" % PROFILE_SHA in text
    assert 'fast_livo2_m6a10_v15_formal_ready.yaml' in text
    assert PHASE in text and TRANSPORT in text
    assert LEGACY_SHA not in text
    for service in (
        '/m6a10/consumer_status', '/m6a10/consumer_ack',
        '/m6a10/consumer_eof', '/m6a10/consumer_finalize',
        '/m6a10/terminal_status', '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ):
        assert service in text
    result = subprocess.run(['bash', '-n', str(WRAPPER)], cwd=ROOT,
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def test_v15_dockerfile_is_exact_v12_base_and_copies_both_feeders():
    text = DOCKERFILE.read_text(encoding='utf-8')
    assert 'ARG FAST_LIVO2_V12_BASE_IMAGE=%s' % BASE_TAG in text
    assert 'FAST_LIVO2_V12_BASE_ID=%s' % BASE_ID in text
    assert 'FROM ${FAST_LIVO2_V12_BASE_IMAGE}' in text
    assert 'COPY scripts/fast_livo2_m6a10_feeder.py' in text
    assert 'fast_livo2_m6a10_feeder_legacy.py' in text
    assert 'COPY scripts/fast_livo2_m6a10_v15_feeder.py' in text
    assert '/runner/scripts/fast_livo2_m6a10_v15_feeder.py' in text
    assert 'COPY scripts/fast_livo2_m6a10_v15_formal_container_run.sh' in text
    assert 'sha256sum /runner/scripts/fast_livo2_m6a10_feeder.py' in text
    assert 'sha256sum /runner/scripts/fast_livo2_m6a10_v15_feeder.py' in text
    assert 'sha256sum /runner/scripts/fast_livo2_m6a10_feeder_legacy.py' in text
    assert PHASE in text and TRANSPORT in text
    assert 'benchmark.fast_livo2.m6a10_network_expectation="none"' in text
    assert 'benchmark.fast_livo2.m6a10_rootfs_expectation="read_only_runtime"' in text
    assert 'benchmark.fast_livo2.m6a10_formal_replay_forbidden="true"' in text
    assert 'docker run' not in text.lower()
    assert 'rosbag' not in text.lower()


def test_v15_build_script_has_one_network_none_build_and_immutable_receipts():
    text = BUILD.read_text(encoding='utf-8')
    assert BUILD.stat().st_mode & 0o111
    assert BASE_TAG in text and BASE_ID in text
    assert FEEDER_SHA in text and LEGACY_SHA in text and PROFILE_SHA in text
    assert text.count('docker build --network none --pull=false') == 1
    assert '--pull=false' in text
    assert 'test ! -e "${path}"' in text
    assert 'RECEIPT_PATH' in text and 'NO_INPUT_RECEIPT_PATH' in text
    assert 'os.O_EXCL' in text
    assert 'os.chmod(str(target), 0o444)' in text
    assert 'formal_replay_forbidden' in text
    assert 'ground_truth_content_opened' in text and 'scorer_invoked' in text
    assert 'docker rm "${CONTAINER_NAME}"' in text
    result = subprocess.run(['bash', '-n', str(BUILD)], cwd=ROOT,
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr


def test_v15_profile_is_candidate_not_authorization_and_binds_transport():
    value = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    assert value['status'] == 'V15_IMAGE_CANDIDATE'
    assert value['formal_replay_forbidden'] is True
    assert value['formal_replay_authorized'] is False
    assert value['formal_replay_started'] is False
    assert value['replay_count'] == 0
    candidate = value['candidate']
    assert candidate['v15_feeder_sha256'] == FEEDER_SHA
    assert candidate['legacy_feeder_sha256'] == LEGACY_SHA
    assert candidate['legacy_cli_preserved'] is True
    assert value['image']['id'] is None
    assert value['image']['status'] == 'BUILD_PENDING'
    assert value['phase']['contract_version'] == PHASE
    assert value['phase']['transport']['contract_version'] == TRANSPORT
    assert value['safety']['input_mounts'] == 0
    assert value['safety']['ground_truth_mount'] is False
    assert value['safety']['scorer_mount'] is False
    assert value['safety']['formal_replay_forbidden'] is True
